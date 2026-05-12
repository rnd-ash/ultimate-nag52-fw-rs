use std::{
    collections::BTreeMap,
    fs,
    path::PathBuf,
    thread::sleep,
    time::{Duration, Instant},
};

use chrono::{Datelike, Utc};
use clap::*;
use clap_num::maybe_hex;
use color_eyre::{
    eyre::{Report, Result},
    owo_colors::OwoColorize,
};
use console::style;
use defmt_decoder::log::{DefmtLoggerType, format::{Formatter, FormatterConfig, HostFormatter}};
use defmt_parser::Level;
use diag_common::{BootloaderStayReason, CAN_ID_DEFMT_LOG, smarteeprom::CodeSectionInfo};
use ecu_diagnostics::{
    DiagError,
    channel::{IsoTPChannel, IsoTPSettings, Packet, PayloadChannel},
    dynamic_diag::{
        DiagServerBasicOptions, DiagServerEmptyLogger, DynamicDiagSession, TimeoutConfig,
    },
    hardware::{Hardware, HardwareScanner},
    kwp2000::{Kwp2000Protocol, KwpCommand, KwpError, KwpSessionType},
};
use elf::abi::PT_LOAD;
use indicatif::{
    FormattedDuration, HumanBytes, HumanDuration, MultiProgress, ProgressBar, ProgressStyle,
};
use object::{
    Endianness, Object, ObjectSection, ObjectSymbol, SectionKind,
    elf::FileHeader32,
    read::elf::{FileHeader, ProgramHeader},
};

use crate::{
    defmt::{DefmtCanIf, DefmtLogEndpoint, MicrosFormattedDuration},
    usb_diag_compat::UsbDiagIface,
};

mod defmt;
mod usb_diag_compat;

const DSU_CRC32_SEED: u32 = 0xEDB88320;

#[cfg(target_os = "linux")]
use ecu_diagnostics::hardware::socketcan::SocketCanScanner;

#[derive(Debug, Clone, ValueEnum)]
pub enum Interface {
    Usb,
    Can,
    CanFast,
}

#[derive(Subcommand, Clone)]
pub enum Command {
    /// Analyze firmware binary for SRAM/Flash usage
    Analyze { file: PathBuf },
    /// Read out ECU identification
    Ident,
    /// Burn production date into the ECU
    /// THIS CAN ONLY BE PERFORMED ONCE!
    BurnDate,
    /// Set chip security bits
    SetSecurity {
        #[clap(long, short, action)]
        enable: bool,
    },
    /// Flash an application binary to the TCU
    Flash {
        #[clap(long, short)]
        bootloader: Option<PathBuf>,
        #[clap(long)]
        application: PathBuf,
        #[clap(short)]
        log: bool,
    },
    /// Read / Dump memory from the TCU to a binary file
    Read {
        #[clap(value_parser=maybe_hex::<u32>)]
        start_address: u32,
        #[clap(value_parser=maybe_hex::<u32>)]
        end_address: u32,
        output_file: PathBuf,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct ElfSegment {
    phys_addr: u32,
    virt_addr: u32,
    size: u32,
    offset_in_elf: u32,
}

#[derive(clap::Parser, Clone)]
pub struct Flasher {
    #[command(subcommand)]
    pub command: Command,
    #[cfg(not(target_os = "windows"))]
    #[clap(value_enum)]
    pub interface: Interface,
    #[cfg(not(target_os = "windows"))]
    #[arg(required_if_eq_any([
        ("interface","Can"),
        ("interface","can"),
        ("interface","Can-fast"),
        ("interface","can-fast")
    ]))]
    pub can_iface: Option<String>,
}

pub const EGS_DIAG_SETTINGS: DiagServerBasicOptions = DiagServerBasicOptions {
    send_id: 0x07E1,
    recv_id: 0x07E9,
    timeout_cfg: TimeoutConfig {
        read_timeout_ms: 5000,
        write_timeout_ms: 5000,
    },
};

fn launch_server_usb(mp: &mut MultiProgress) -> Result<DynamicDiagSession, Report> {
    let mut next_bar = mp.add(ProgressBar::new_spinner());
    let spinner_style = ProgressStyle::with_template("{prefix:.bold.dim} {spinner} {wide_msg}")
        .unwrap()
        .tick_chars("⠁⠂⠄⡀⢀⠠⠐⠈ ");
    next_bar.set_style(spinner_style);
    next_bar.enable_steady_tick(Duration::from_millis(100));
    next_bar = next_bar.with_message("Waiting for device to be available");
    let serial = UsbDiagIface::new()?;
    let mut channel = Box::new(serial) as Box<dyn IsoTPChannel>;

    let egs_isotp_opts: IsoTPSettings = IsoTPSettings {
        block_size: 0,
        st_min: 0,
        extended_addresses: None,
        pad_frame: true,
        can_speed: 500_000,
        can_use_ext_addr: false,
    };

    channel.set_iso_tp_cfg(egs_isotp_opts)?;
    channel.set_ids(0x07E1, 0x07E9)?;
    channel.open()?;

    let server = DynamicDiagSession::new(
        Kwp2000Protocol::default(),
        channel,
        EGS_DIAG_SETTINGS,
        None,
        DiagServerEmptyLogger {},
    )?;
    mp.remove(&next_bar);
    Ok(server)
}

fn create_server(
    fast_mode: &mut bool,
    args: &Flasher,
    mp: &mut MultiProgress,
) -> Result<DynamicDiagSession, Report> {
    #[cfg(target_os = "linux")]
    let server = match args.interface {
        Interface::Usb => launch_server_usb(mp),
        Interface::Can => {
            *fast_mode = false;
            launch_server_isotp(&args.can_iface.clone().unwrap(), false)
        }
        Interface::CanFast => {
            *fast_mode = true;
            launch_server_isotp(&args.can_iface.clone().unwrap(), true)
        }
    }?;
    #[cfg(not(target_os = "linux"))]
    let server = launch_server_usb()?;
    Ok(server)
}

#[cfg(target_os = "linux")]
fn launch_server_isotp(can_iface_name: &str, fast: bool) -> Result<DynamicDiagSession, Report> {
    use ecu_diagnostics::dynamic_diag::DiagServerAdvancedOptions;

    let mut socket_can = SocketCanScanner::new().open_device_by_name(can_iface_name)?;
    let mut channel = socket_can.create_iso_tp_channel()?;

    let egs_isotp_opts: IsoTPSettings = IsoTPSettings {
        block_size: if fast { 0 } else { 0x20 },
        st_min: if fast { 0 } else { 10 },
        extended_addresses: None,
        pad_frame: true,
        can_speed: 500_000,
        can_use_ext_addr: false,
    };
    let adv_opts = DiagServerAdvancedOptions {
        global_tp_id: 0,
        tester_present_interval_ms: 2000,
        tester_present_require_response: true,
        global_session_control: false,
        tp_ext_id: None,
        command_cooldown_ms: 0,
    };

    channel.set_iso_tp_cfg(egs_isotp_opts)?;
    channel.set_ids(0x07E1, 0x07E9)?;
    channel.open()?;

    let mut server = DynamicDiagSession::new(
        Kwp2000Protocol::default(),
        channel,
        EGS_DIAG_SETTINGS,
        Some(adv_opts),
        DiagServerEmptyLogger {},
    )?;
    server.set_retry_count(5);
    Ok(server)
}

fn next_spinner(
    mp: &MultiProgress,
    last_bar: Option<ProgressBar>,
    stage: u32,
    out_of: u32,
) -> ProgressBar {
    if let Some(last_bar) = last_bar {
        let old_msg = last_bar.message();
        last_bar.finish_with_message(format!("{old_msg} {}", style("✔").green()));
    }

    let next_bar = mp.add(ProgressBar::new_spinner());
    let spinner_style = ProgressStyle::with_template("{prefix:.bold.dim} {spinner} {wide_msg}")
        .unwrap()
        .tick_chars("⠁⠂⠄⡀⢀⠠⠐⠈ ");
    next_bar.set_style(spinner_style);
    next_bar.set_prefix(format!("[{stage}/{out_of}]"));
    next_bar.enable_steady_tick(Duration::from_millis(100));
    next_bar
}

fn read(
    mp: &MultiProgress,
    dest_file: &PathBuf,
    start_addr: u32,
    end_addr: u32,
    server: DynamicDiagSession,
    fast_mode: bool,
) -> Result<(), Report> {
    let spinner = next_spinner(&mp, None, 1, 3);
    spinner.set_message("Enter extended mode");
    // Now start the command chain
    if fast_mode {
        server.send_byte_array_with_response(
            &[
                KwpCommand::StartDiagnosticSession.into(),
                KwpSessionType::Reprogramming.into(),
                0,
                0,
            ],
            None,
        )?;
    } else {
        server.kwp_set_session(KwpSessionType::ExtendedDiagnostics.into())?;
    }
    let spinner = next_spinner(&mp, Some(spinner), 2, 3);
    spinner.set_message(format!(
        "Reading memory ({})",
        HumanBytes((end_addr - start_addr) as u64)
    ));
    let mut v: Vec<u8> = Vec::new();
    let total_bytes = end_addr - start_addr;

    let pb = mp
        .add(ProgressBar::new(total_bytes as u64).with_message("Reading"))
        .with_style(
            ProgressStyle::with_template(
                "{percent}% [{bar:40.cyan/blue}] {msg} {decimal_bytes_per_sec} ETA: {eta}",
            )
            .unwrap()
            .progress_chars("##-"),
        );

    while (v.len() as u32) < total_bytes {
        let max = std::cmp::min(250, total_bytes - v.len() as u32);
        pb.set_message(format!(
            "0x{:08X}-0x{:08X}",
            start_addr + v.len() as u32,
            start_addr + v.len() as u32 + max
        ));
        let mut req = vec![0x23, max as u8];
        req.extend_from_slice(&(start_addr + v.len() as u32).to_le_bytes());
        let resp = server.send_byte_array_with_response(&req, None)?;
        v.extend_from_slice(&resp[1..]);
        pb.set_position(v.len() as u64);
    }
    pb.finish_with_message(format!("{}", style("✔").green()));
    let spinner = next_spinner(&mp, Some(spinner), 3, 3);
    spinner.set_message("Writing memory to file");
    std::fs::write(dest_file, v)?;
    spinner.finish_with_message(format!("{} {}", spinner.message(), style("✔").green()));
    Ok(())
}

// Limit on flash size
fn analyze(file: &PathBuf, flash_max: u64) -> Result<(), Report> {
    let binary_bytes = fs::read(file)?;
    let elf = object::File::parse(&*binary_bytes)?;
    let mut flash_bytes: usize = 0;
    let mut ram_bytes: usize = 0;
    let mut high_ram_watermark: u32 = 0;
    let mut defmt_size: u64 = get_defmt_bytes(file).len() as u64;
    for section in elf.sections() {
        if section.kind() == SectionKind::Other
            || section.kind() == SectionKind::Metadata
            || section.kind() == SectionKind::OtherString
        {
            continue;
        }
        if section.address() < 0x100000 {
            // Flash
            flash_bytes += section.size() as usize;
        } else if section.address() >= 0x20000000 && section.address() <= 0x2003FFFF {
            // RAM
            ram_bytes += section.size() as usize;
            let max = (section.size() + section.address()) as u32;
            high_ram_watermark = high_ram_watermark.max(max);
        }
    }
    let ram_alloc = high_ram_watermark - 0x20000000;
    let pb_flash = ProgressBar::new(flash_max).with_style(
        ProgressStyle::with_template(
            "Flash usage: [{bar:40.cyan/blue}] {percent}% ({decimal_bytes}/{total_bytes})",
        )
        .unwrap()
        .progress_chars("##-"),
    );
    pb_flash.set_position(flash_bytes as u64);
    pb_flash.abandon();
    let pb_qspi = ProgressBar::new(1024 * 256).with_style(
        ProgressStyle::with_template(
            " QSPI usage: [{bar:40.cyan/blue}] {percent}% ({decimal_bytes}/{total_bytes})",
        )
        .unwrap()
        .progress_chars("##-"),
    );
    pb_qspi.set_position(defmt_size as u64);
    pb_qspi.abandon();
    let pb_ram = ProgressBar::new(256 * 1024).with_style(
        ProgressStyle::with_template(
            "  RAM usage: [{bar:40.cyan/blue}] {percent}% ({decimal_bytes}/{total_bytes})",
        )
        .unwrap()
        .progress_chars("##-"),
    );
    pb_ram.set_position(ram_bytes as u64);
    pb_ram.abandon();
    let pb_ram_watermark = ProgressBar::new(256 * 1024).with_style(
        ProgressStyle::with_template(
            "  RAM alloc: [{bar:40.cyan/blue}] {percent}% ({decimal_bytes}/{total_bytes})",
        )
        .unwrap()
        .progress_chars("##-"),
    );
    pb_ram_watermark.set_position(ram_alloc as u64);
    pb_ram_watermark.abandon();
    Ok(())
}

fn flash(
    mp: &MultiProgress,
    file: &PathBuf,
    server: &mut DynamicDiagSession,
    fast_mode: bool,
    is_bl: bool,
) -> Result<(), Report> {
    const PRE_END_ADDR: u64 = 1024 * 8;
    const BL_END_ADDR: u64 = 1024 * 128;
    let max_size = if is_bl {
        BL_END_ADDR - PRE_END_ADDR
    } else {
        (1024 * 1024) - BL_END_ADDR
    };
    analyze(file, max_size)?;
    let binary_bytes = fs::read(file)?;
    let binary = FileHeader32::<Endianness>::parse(&*binary_bytes)?;
    let endian = binary.endian()?;

    let mut segments = Vec::new();

    for segment in binary.program_headers(binary.endian()?, &*binary_bytes)? {
        let p_paddr: u64 = segment.p_paddr(endian).into();
        let p_vaddr: u64 = segment.p_vaddr(endian).into();
        let segment_data = segment
            .data(endian, &*binary_bytes)
            .map_err(|_| Report::msg("Failed to access data for ELF segment"))?;
        if !segment_data.is_empty() {
            if segment.p_type(endian) == PT_LOAD {
                let (segment_offset, segment_filesize) = segment.file_range(endian);
                segments.push(ElfSegment {
                    phys_addr: p_paddr as u32,
                    virt_addr: p_vaddr as u32,
                    size: segment_filesize as u32,
                    offset_in_elf: segment_offset as u32,
                });
            } else {
                println!("{segment:?}");
            }
        }
    }

    segments.sort_by(|x, y| x.phys_addr.cmp(&y.phys_addr));
    let start_address = segments[0].phys_addr;
    let last = segments.last().unwrap();
    let end_addr = last.phys_addr + last.size;
    let to_flash = end_addr - start_address;
    assert!(start_address % 8192 == 0);
    let mut array = vec![0xFFu8; to_flash as usize];
    for seg in segments {
        let offset = seg.phys_addr as usize - start_address as usize;
        array[offset..offset + seg.size as usize].copy_from_slice(
            &binary_bytes
                [seg.offset_in_elf as usize..seg.size as usize + seg.offset_in_elf as usize],
        );
    }
    let mut num_pages = array.len() / 8192;
    if array.len() % 8192 != 0 {
        num_pages += 1;
    }
    let spinner = next_spinner(&mp, None, 1, 6);
    spinner.set_message("Enter programming mode");
    // Now start the command chain
    if fast_mode {
        server.send_byte_array_with_response(
            &[
                KwpCommand::StartDiagnosticSession.into(),
                KwpSessionType::Reprogramming.into(),
                0,
                0,
            ],
            None,
        )?;
    } else {
        server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    }
    std::thread::sleep(Duration::from_millis(1000)); // Allow the MCU to reset to bootloader
    let spinner = next_spinner(&mp, Some(spinner), 2, 6);
    spinner.set_message(format!(
        "Erasing flash ({} from 0x{:08X})",
        HumanBytes((num_pages * 8192) as u64),
        start_address
    ));

    let mut erase_cmd = [0; 8];
    erase_cmd[0] = 0x31;
    erase_cmd[1] = 0xE0;
    erase_cmd[2..6].copy_from_slice(&(start_address as u32).to_le_bytes());
    erase_cmd[6..8].copy_from_slice(&(num_pages as u16).to_le_bytes());
    server.send_byte_array_with_response(&erase_cmd, None)?;
    let mut e_counter = 0;
    loop {
        match server.send_byte_array_with_response(
            &[
                KwpCommand::RequestRoutineResultsByLocalIdentifier.into(),
                0xE0,
            ],
            None,
        ) {
            Ok(res) => {
                if res[2] == 0x00 {
                    break;
                } else {
                    return Err(Report::msg("Flash erase failed"));
                }
            }
            Err(DiagError::ECUError { code, def }) => {
                if code == KwpError::RoutineNotComplete as u8 {
                    // Waiting
                    sleep(Duration::from_millis(500));
                } else {
                    return Err(DiagError::ECUError { code, def }.into());
                }
            }
            Err(e) => {
                e_counter += 1;
                // Can happen after reboot
                if fast_mode {
                    server.send_byte_array_with_response(
                        &[
                            KwpCommand::StartDiagnosticSession.into(),
                            KwpSessionType::Reprogramming.into(),
                            0,
                            0,
                        ],
                        None,
                    )?;
                } else {
                    server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
                }
                server.send_byte_array_with_response(&erase_cmd, None)?;
                if e_counter == 2 {
                    return Err(e.into());
                }
            }
        }
    }
    // Flash erase completed
    let spinner = next_spinner(&mp, Some(spinner), 3, 6);
    spinner.set_message("Preparing download");
    let mut download_req = vec![KwpCommand::RequestDownload.into()];
    download_req.extend_from_slice(&(start_address as u32).to_le_bytes());
    download_req.push(0x00); // Fmt
    download_req.extend_from_slice(&(array.len() as u32).to_le_bytes());
    server.send_byte_array_with_response(&download_req, None)?;
    let mut counter: u8 = 0;
    const MAX_COPY: usize = 1024;
    let mut block = [0; MAX_COPY + 2];
    let mut addr = 0;

    let spinner = next_spinner(&mp, Some(spinner), 4, 6);
    spinner.set_message(format!(
        "Transfering data  ({})",
        HumanBytes(array.len() as u64)
    ));
    let pb = mp
        .add(ProgressBar::new(array.len() as u64).with_message("Flashing"))
        .with_style(
            ProgressStyle::with_template(
                "{percent}% [{bar:40.cyan/blue}] {msg} {decimal_bytes_per_sec} ETA: {eta}",
            )
            .unwrap()
            .progress_chars("##-"),
        );
    while addr < array.len() {
        let max_copy = core::cmp::min(MAX_COPY, array.len() - addr);
        block[0] = KwpCommand::TransferData.into();
        block[1] = counter;
        block[2..2 + max_copy].copy_from_slice(&array[addr..addr + max_copy]);
        pb.set_position(addr as u64);
        server.send_byte_array_with_response(&block[..max_copy + 2], None)?;
        addr += max_copy;
        counter = counter.wrapping_add(1);
    }
    pb.finish_with_message(format!("{}", style("✔").green()));
    mp.remove(&pb);
    let spinner = next_spinner(&mp, Some(spinner), 5, 6);
    spinner.set_message("Verifying flashed data");
    // Start flash check routine
    let mut hasher = crc32fast::Hasher::new_with_initial(DSU_CRC32_SEED);
    hasher.reset();
    hasher.update(&array);
    let targ_crc = hasher.finalize();
    let mut buf = vec![0x31, 0xE1];
    let start = start_address as u32;
    buf.extend_from_slice(&targ_crc.to_le_bytes());
    buf.extend_from_slice(&start.to_le_bytes());
    buf.extend_from_slice(&(array.len() as u32).to_le_bytes());
    let response = server.send_byte_array_with_response(&buf, None)?;
    if response[2] == 0x00 {
        return Err(Report::msg("Flash CRC compare failed"));
    }

    // Reset ECU
    let spinner = next_spinner(&mp, Some(spinner), 6, 6);
    spinner.set_message("Resetting ECU");
    server.send_byte_array_with_response(&[KwpCommand::ECUReset.into(), 0x01], None)?;
    spinner.finish_with_message(format!("{} {}", spinner.message(), style("✔").green()));
    Ok(())
}

fn ident(server: DynamicDiagSession) -> Result<(), Report> {
    let mut map: BTreeMap<&'static str, Option<String>> = BTreeMap::new();
    if let Ok(ident) = server.kwp_read_daimler_identification() {
        map.insert(
            "ECU Production date",
            Some(ident.get_production_date_pretty()),
        );
        map.insert(
            "ECU Software date (WW/YY)",
            Some(ident.get_software_date_pretty()),
        );
    } else {
        map.insert("ECU Production date", None);
        map.insert("ECU Software date", None);
    }

    if let Ok(ident) = server.kwp_read_ecu_serial_number() {
        let mut res = String::new();
        for b in ident {
            res.push_str(&format!("{b:02X?}"));
        }
        map.insert("ECU Serial Number", Some(res));
    } else {
        map.insert("ECU Serial Number", None);
    }

    let ident = server.kwp_read_daimler_mmc_identification()?;
    map.insert("HW Version", Some(format!("{}", ident.hw_version)));
    map.insert("SW Version", Some(format!("{}", ident.sw_version)));

    let s = if ident.diag_info.is_boot_sw() {
        style("Bootloader").bold().red()
    } else {
        style("Application")
    };
    map.insert("Software type", Some(format!("{s}")));

    let dbg = if ident.diag_info.is_production_ecu() {
        style("No")
    } else {
        style("Yes").bold().red()
    };

    map.insert("Debug mode SW", Some(format!("{dbg}")));

    let mut panic_msg: Option<String> = None;
    let mut panic_location: Option<(String, u32, u32)> = None;

    if ident.diag_info.is_boot_sw() {
        if let Ok(res) = server.send_byte_array_with_response(&[0x21, 0xE2], None) {
            let txt = match BootloaderStayReason::from(res[2]) {
                BootloaderStayReason::None => style("Diagnostics").green(),
                BootloaderStayReason::ResetCount => style("Quick reboot count exceeded").yellow(),
                BootloaderStayReason::Watchdog => style("Watchdog triggered").red(),
                BootloaderStayReason::Panic => style("Application panicked (See below)").red(),
                BootloaderStayReason::RamFailure => style("RAM Test failure").red(),
                BootloaderStayReason::MagicPin => style("Magic pin held high").green(),
                BootloaderStayReason::ProductionInfoNotSet => {
                    style("Board production info not burned").yellow()
                }
                BootloaderStayReason::AppInvalid => {
                    style("Application invalid or flashing not completed").yellow()
                }
                BootloaderStayReason::Unkown => style("Unknown").red(),
                BootloaderStayReason::Diagnostics => style("Diagnostics").green(),
            };
            map.insert("In Bootloader reason", Some(format!("{}", txt)));

            if BootloaderStayReason::from(res[2]) == BootloaderStayReason::Panic {
                if let Ok(panic_msg_res) = server.send_byte_array_with_response(&[0x21, 0xE3], None)
                {
                    let msg = String::from_utf8_lossy(&panic_msg_res[2..]);
                    panic_msg = Some(msg.to_string());
                }
                if let Ok(location_msg_res) =
                    server.send_byte_array_with_response(&[0x21, 0xE4], None)
                {
                    let column = u32::from_le_bytes(location_msg_res[2..6].try_into().unwrap());
                    let line = u32::from_le_bytes(location_msg_res[6..10].try_into().unwrap());
                    let file = String::from_utf8_lossy(&location_msg_res[10..]);
                    panic_location = Some((file.to_string(), line, column));
                }
            }
        } else {
            map.insert("In Bootloader reason", None);
        }
    }

    println!(
        "{}",
        style("Identification information").bold().bright_blue()
    );
    let mut padding = 0;
    for k in map.keys() {
        padding = padding.max(k.len());
    }
    padding += 1;

    for (k, v) in map {
        println!(
            "{: <padding$}: {}",
            style(k).bold(),
            v.map(|x| style(x).green())
                .unwrap_or(style("REFUSED".into()).bold().red())
        );
    }

    if panic_location.is_some() || panic_msg.is_some() {
        println!(
            "\n{}",
            style("Application panic information").bold().bright_red()
        );
        if let Some(msg) = panic_msg {
            println!("{: <padding$}: {}", style("Panic message").bold(), msg);
        }
        if let Some((file, line, col)) = panic_location {
            println!(
                "{: <padding$}: {}:{}:{}",
                style("Panic location").bold(),
                file,
                line,
                col
            );
        }
    } else {
        // Print more detailed info if required
        if let Ok(ident) = server.send_byte_array_with_response(&[0x1A, 0x8A], None) {
            println!(
                "\n{}",
                style("Software detailed information")
                    .bold()
                    .bright_purple()
            );

            let mut offset = 2;
            for (idx, cat) in ["Preloader block", "Bootloader block", "Application block"]
                .iter()
                .enumerate()
            {
                let blob = ident[offset..offset + std::mem::size_of::<CodeSectionInfo>()].as_ptr()
                    as *const u8;
                let cast = blob as *const CodeSectionInfo;
                let code_sec = unsafe { cast.as_ref().unwrap() };
                offset += std::mem::size_of::<CodeSectionInfo>();
                let flashed = !code_sec.name.contains(&0xFF);
                let s = if flashed {
                    style("Present").bold().green()
                } else {
                    style("Not flashed").bold().red()
                };
                let (wall, arrow) = if idx == 2 {
                    (" ", "└─")
                } else {
                    ("│", "├─")
                };
                println!("{arrow}{}: {}", style(cat).bold(), s);
                if !flashed {
                    continue;
                }
                // Print information
                const PADDING: usize = 20;
                let name = String::from_utf8_lossy(&code_sec.name);
                let sha = String::from_utf8_lossy(&code_sec.git_sha);

                println!(
                    "{wall} ├─{: <PADDING$}: {}",
                    style("Identified name").bold().dim(),
                    style(name).green().dim(),
                );

                println!(
                    "{wall} ├─{: <PADDING$}: {}",
                    style("Git SHA").bold().dim(),
                    style(sha).green().dim(),
                );

                println!(
                    "{wall} ├─{: <PADDING$}: {}",
                    style("Version").bold().dim(),
                    style(format!(
                        "{}.{}.{}",
                        code_sec.version_major, code_sec.version_minor, code_sec.version_patch
                    ))
                    .green()
                    .dim(),
                );

                println!(
                    "{wall} ├─{: <PADDING$}: {}",
                    style("Rustc version").bold().dim(),
                    style(format!(
                        "{}.{}.{}",
                        code_sec.rustc_version_major,
                        code_sec.rustc_version_minor,
                        code_sec.rustc_version_patch
                    ))
                    .green()
                    .dim(),
                );

                println!(
                    "{wall} ├─{: <PADDING$}: {}",
                    style("Compile date").bold().dim(),
                    style(format!(
                        "{}/{}/20{:02} (Week {})",
                        code_sec.compile_day,
                        code_sec.compile_month,
                        code_sec.compile_year,
                        code_sec.compile_month
                    ))
                    .green()
                    .dim(),
                );

                let dbg_txt = if code_sec.is_debug == 0 {
                    style("No").green().dim()
                } else {
                    style("Yes").yellow().dim()
                };

                if idx > 0 {
                    // Grab CRC too
                    let start = 101 + ((idx - 1) * 4);
                    let crc = u32::from_le_bytes(ident[start..start + 4].try_into().unwrap());
                    let crc_txt = if crc == 0xFFFF_FFFF || crc == 0 {
                        // Debug flash
                        style("Not present - Flashed via debugger".to_string())
                            .yellow()
                            .dim()
                    } else {
                        style(format!("0x{:08X}", crc)).green().dim()
                    };
                    println!(
                        "{wall} ├─{: <PADDING$}: {crc_txt}",
                        style("Fingerprint").bold().dim()
                    );
                }

                println!(
                    "{wall} └─{: <PADDING$}: {dbg_txt}",
                    style("Debug build").bold().dim(),
                );

                if idx != 2 {
                    println!("{wall}");
                }
            }
        }
    }

    Ok(())
}

fn burn_date(server: DynamicDiagSession) -> Result<(), Report> {
    server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    let date = Utc::now();
    let mut req = [
        KwpCommand::StartRoutineByLocalIdentifier as u8,
        0x24,
        0,
        0,
        0,
        0,
    ];
    req[2] = date.day() as u8; // Day
    req[3] = date.iso_week().week() as u8; // Week
    req[4] = date.month() as u8; // Month
    req[5] = (date.year() % 100) as u8; // Year
    server.send_byte_array_with_response(&req, None)?;
    println!(
        "Burnt production date: {}/{}/{} (Week {})",
        req[2], req[4], req[5], req[3]
    );
    Ok(())
}

fn set_security_lock(server: DynamicDiagSession, en: bool) -> Result<(), Report> {
    server.kwp_set_session(KwpSessionType::Reprogramming.into())?;
    server.send_byte_array_with_response(
        &[
            KwpCommand::StartRoutineByLocalIdentifier.into(),
            0xFE,
            en as u8,
        ],
        None,
    )?;
    Ok(())
}

fn get_defmt_bytes(path: &PathBuf) -> Vec<u8> {
    let elf_bytes = fs::read(path).unwrap();
    let tab = defmt_decoder::Table::parse(&elf_bytes).ok().flatten();
    if let Some(table) = tab {
        let res = postcard::to_allocvec(&table).unwrap();
        lz4_flex::compress(&res)
    } else {
        vec![]
    }
}

fn attach_log(path: &PathBuf, ty: Interface, name: Option<String>) -> Result<(), Report> {
    let elf_bytes = fs::read(path).unwrap();
    let table = defmt_decoder::Table::parse(&elf_bytes)
        .map_err(|e| Report::msg(e.to_string()))?
        .ok_or_else(|| Report::msg(".defmt table not found"))?;
    let locations = table.get_locations(&elf_bytes).ok();

    let server: Box<dyn DefmtLogEndpoint> = match ty {
        Interface::Usb => Box::new(UsbDiagIface::new().unwrap()),
        Interface::Can | Interface::CanFast => {
            let scan_iface = name.unwrap();
            let logger = DefmtCanIf::new(&scan_iface).unwrap();
            Box::new(logger)
        }
    };
    loop {
        while let Some(frame) = server.read_msg() {
            if let Some(decoded) = defmt::decode_msg(&frame, &table, &locations) {
                let level_txt = match decoded.level {
                    Some(Level::Info) => format!("{}", "INFO".green().bold()),
                    Some(Level::Warn) => format!("{}", "WARN".yellow().bold()),
                    Some(Level::Error) => format!("{}", "ERROR".red().bold()),
                    Some(Level::Trace) => format!("{}", "TRACE".bold()),
                    Some(Level::Debug) => format!("{}", "DEBUG".bold()),
                    None => format!("{}", "PRINT".bold()),
                };

                let ts_txt = if let Some(ts) = decoded.ts {
                    format!("{}", MicrosFormattedDuration(ts))
                } else {
                    "".to_string()
                };

                println!("[{:-<10} {}] {}", level_txt, ts_txt, decoded.msg)
            } else {
                println!("Decode err: {frame:02X?}")
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

fn main() -> Result<()> {
    env_logger::init();
    color_eyre::install()?;
    let start_timer = Instant::now();
    let args = Flasher::parse();

    if let Command::Analyze { file } = args.command.clone() {
        analyze(&file, 1024 * 1024)?;
        attach_log(&file, args.interface, args.can_iface);
        return Ok(());
    }

    let mut fast_mode = false;
    let mut mp = MultiProgress::new();
    let mut server = create_server(&mut fast_mode, &args, &mut mp)?;
    let res = match &args.command {
        Command::Flash {
            bootloader,
            application,
            log,
        } => {
            let has_bootloader = bootloader.is_some();
            if let Some(loader) = bootloader {
                println!(
                    "{}",
                    style("Flashing bootloader (Stage 1/2)").bold().green()
                );
                flash(&mp, loader, &mut server, fast_mode, true)?;
            }
            drop(mp);
            mp = MultiProgress::new();
            if has_bootloader {
                println!(
                    "{}",
                    style("Flashing application (Stage 2/2)").bold().green()
                );
                // Restart the server to drain buffers etc
                let _ = server.release();
                std::thread::sleep(Duration::from_millis(1000));
                server = create_server(&mut fast_mode, &args, &mut mp).unwrap();
            } else {
                println!("{}", style("Flashing application").bold().green());
            }

            flash(&mp, application, &mut server, fast_mode, false)?;
            if *log {
                // Switch to log mode
                let log_mode = match args.interface {
                    Interface::Usb => 2,
                    Interface::Can | Interface::CanFast => 1,
                };
                server.kwp_set_session(KwpSessionType::ExtendedDiagnostics.into())?;
                server.send_byte_array_with_response(&[0x30, 0xF0, 0x07, log_mode], None)?;
                drop(server);
                attach_log(application, args.interface, args.can_iface);
            }
            Ok(())
        }

        Command::Read {
            start_address,
            end_address,
            output_file,
        } => read(
            &mp,
            output_file,
            *start_address,
            *end_address,
            server,
            fast_mode,
        ),
        Command::Ident => ident(server),
        Command::BurnDate => burn_date(server),
        Command::SetSecurity { enable } => set_security_lock(server, *enable),
        Command::Analyze { .. } => {
            unreachable!()
        }
    };
    if res.is_err() {
        mp.clear()?;
    }
    res?;

    println!(
        "{}",
        style(format!(
            "Completed in {}",
            HumanDuration(start_timer.elapsed())
        ))
        .bold()
        .green()
    );
    Ok(())
}
