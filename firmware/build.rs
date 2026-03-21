use candb_codegen::codegen_all_dbs;
use chrono::Datelike;
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use vergen::Emitter;
use vergen_git2::Git2Builder;
use std::fmt::Write as _;

const MEM_F_NAME: &str = "../memory_full.x";

fn main() {
    let mem_x =  std::fs::read_to_string(MEM_F_NAME).unwrap();
    let mut mem_output = String::new();
    for line in mem_x.lines() {
        if line.contains("FLASH_APP") {
            let _ = writeln!(&mut mem_output, "{}", line.replace("#FLASH_APP", "FLASH"));
        } else {
            let _ = writeln!(&mut mem_output, "{}", line);
        }
    }


    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(mem_output.as_bytes())
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={}", MEM_F_NAME);
    println!("cargo:rerun-if-changed=build.rs");

    let vergen = Git2Builder::all_git().unwrap();

    // Can data parsing
    println!("cargo::rerun-if-changed=can_data/custom_can.txt");
    println!("cargo::rerun-if-changed=can_data/egs51.txt");
    println!("cargo::rerun-if-changed=can_data/egs52.txt");
    println!("cargo::rerun-if-changed=can_data/egs53.txt");
    println!("cargo::rerun-if-changed=can_data/hfm.txt");
    println!("cargo::rerun-if-changed=can_data/slave_mode.txt");

    codegen_all_dbs(PathBuf::from("can_data/"), PathBuf::from("src/can/data/"));

    // Generate our build timestamps for bootloader info header
    let time = chrono::Utc::now();
    println!("cargo::rustc-env=BUILD_YEAR={}", time.year() - 2000);
    println!("cargo::rustc-env=BUILD_MONTH={}", time.month());
    println!("cargo::rustc-env=BUILD_WEEK={}", time.iso_week().week());
    println!("cargo::rustc-env=BUILD_DAY={}", time.day());

    // Grab rust version info
    let v = rustc_version::version().unwrap();
    println!("cargo::rustc-env=RUSTC_VER_MAJOR={}", v.major);
    println!("cargo::rustc-env=RUSTC_VER_MINOR={}", v.minor);
    println!("cargo::rustc-env=RUSTC_VER_PATCH={}", v.patch);

    Emitter::default()
        .add_instructions(&vergen)
        .unwrap()
        .emit()
        .unwrap()
}
