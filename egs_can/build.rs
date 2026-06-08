use std::{fs, path::PathBuf};

use candb_codegen::codegen_db;

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
    optionally_build_candb("../can_data/custom_can.txt", "src/can_matrix/custom_can");
    optionally_build_candb("../can_data/egs51.txt", "src/can_matrix/egs_51");
    optionally_build_candb("../can_data/egs52.txt", "src/can_matrix/egs_52");
    optionally_build_candb("../can_data/egs53.txt", "src/can_matrix/egs_53");
    optionally_build_candb("../can_data/hfm.txt", "src/can_matrix/hfm_can");
    optionally_build_candb("../can_data/slave_mode.txt", "src/can_matrix/slave_mode");
}
