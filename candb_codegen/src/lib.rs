use std::{fs, path::PathBuf};

use convert_case::{Case, Casing};

use crate::code_gen::CodeGenerator;

pub(crate) mod code_gen;
pub(crate) mod parser;

pub fn codegen_db(f_in: impl Into<PathBuf>, dir_out: impl Into<PathBuf>) {
    let f_in_pb = f_in.into();
    let dir_out_pb = dir_out.into();
    if f_in_pb.is_file() {
        // File
        let f_name = f_in_pb.file_name().unwrap().to_str().unwrap();
        if f_name.ends_with("txt") {
            // Can data file
            let can_name = dir_out_pb
                .file_name()
                .unwrap()
                .to_str()
                .unwrap()
                .to_string();

            std::fs::create_dir_all(&dir_out_pb).unwrap();

            let in_file = fs::read(f_in_pb).unwrap();
            let s = String::from_utf8(in_file).unwrap();
            let mut candb_parser = parser::CanDbParser::default();
            candb_parser.parse_file(s);
            let code_generator = CodeGenerator::new(candb_parser.ecus, dir_out_pb);
            code_generator.code_gen(&can_name).unwrap();
        } else {
            panic!("Not a CAN database")
        }
    }
}
