use std::{fs, path::PathBuf};

use convert_case::{Case, Casing};

use crate::code_gen::CodeGenerator;

pub (crate) mod parser;
pub (crate) mod code_gen;

pub fn codegen_all_dbs(dir_in: PathBuf, dir_out: PathBuf) {
    for path in dir_in.read_dir().unwrap() {
        let p = path.unwrap();
        let f_ty = p.file_type().unwrap();
        if f_ty.is_file() {
            // File
            let f_name = p.file_name().into_string().unwrap();
            if f_name.ends_with("txt") {
                // Can data file
                let can_name = f_name.split(".").next().unwrap().to_case(Case::Snake);
                let mut out_dir = dir_out.clone();
                out_dir.push(&can_name);

                std::fs::create_dir_all(&out_dir).unwrap();
                for file in out_dir.read_dir().unwrap() {
                    std::fs::remove_file(file.unwrap().path()).unwrap()
                }

                let in_file = fs::read(p.path()).unwrap();
                let s = String::from_utf8(in_file).unwrap();
                let mut candb_parser = parser::CanDbParser::default();
                candb_parser.parse_file(s);
                let code_generator = CodeGenerator::new(candb_parser.ecus, out_dir);
                code_generator.code_gen(&can_name).unwrap();


            } else {
                panic!("Not a CAN database")
            }

        }
    }
}