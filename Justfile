# Build CLI Flasher app

build_cli_app:
    cargo build --release --package flasher

[working-directory: 'bootloader']
run_bl *args:
    just build_cli_app
    cargo build --release --features=skip-app-check
    ./../target/release/flasher {{args}} flash --bootloader ../target/thumbv7em-none-eabihf/release/bootloader --compress

# Flash firmware
[working-directory: 'firmware']
run_fw *args:
    just build_cli_app
    cargo build --release
    ./../target/release/flasher {{args}} flash --application ../target/thumbv7em-none-eabihf/release/firmware --compress
    ./../target/release/flasher {{args}} monitor

# Flash and run firmware
[working-directory: 'firmware']
run_fw_probe *args:
    just build_cli_app
    cargo build --release
    ./../target/release/flasher {{args}} flash --application ../target/thumbv7em-none-eabihf/release/firmware --compress
    probe-rs attach --protocol swd ../target/thumbv7em-none-eabihf/release/firmware

ident *args:
    just build_cli_app
    ./target/release/flasher {{args}} ident

dump_ram *args:
    just build_cli_app
    ./target/release/flasher {{args}} read 0x20000000 0x20040000 dump_ram.bin

dump_qspi *args:
    just build_cli_app
    ./target/release/flasher {{args}} read 0x04000000 0x05000000 dump_qspi.bin

monitor *args:
    just build_cli_app
    ./target/release/flasher {{args}} monitor
