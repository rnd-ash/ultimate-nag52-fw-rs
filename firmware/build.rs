use candb_codegen::codegen_db;
use chrono::Datelike;
use std::fmt::Write as _;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::{env, fs};
use vergen::Emitter;
use vergen_git2::Git2Builder;

const MEM_F_NAME: &str = "../memory_full.x";

fn optionally_build_candb(db: &str, folder: &str) {
    let mut p: PathBuf = folder.into();
    let generate = if !p.exists() {
        // CAN DB Folder doesn't exist, so we must generate the DB
        true
    } else {
        // Compare modification timestamps
        p.push("mod.rs"); // To look at the metadata of mod.rs
        let creation_time = fs::metadata(p).unwrap().modified().unwrap();
        let db_modified_time = fs::metadata(db).unwrap().modified().unwrap();
        db_modified_time > creation_time
    };
    if generate {
        codegen_db(db, folder);
    }
}

fn main() {
    let mem_x = std::fs::read_to_string(MEM_F_NAME).unwrap();
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

    optionally_build_candb("can_data/custom_can.txt", "src/can/data/custom_can");
    optionally_build_candb("can_data/egs51.txt", "src/can/data/egs_51");
    optionally_build_candb("can_data/egs52.txt", "src/can/data/egs_52");
    optionally_build_candb("can_data/egs53.txt", "src/can/data/egs_53");
    optionally_build_candb("can_data/hfm.txt", "src/can/data/hfm_can");
    optionally_build_candb("can_data/slave_mode.txt", "src/can/data/slave_mode");

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
