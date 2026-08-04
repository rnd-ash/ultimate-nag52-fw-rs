# Build CLI Flasher app

[working-directory: 'flasher']
build_cli_app:
    cargo build --release

[working-directory: 'bootloader']
run_bl *args:
    just build_cli_app
    cargo build --release --features=skip-app-check
    ./../flasher/target/release/flasher {{args}} flash --bootloader ../target/thumbv7em-none-eabihf/release/bootloader --compress

# Flash and run firmware
[working-directory: 'firmware']
run_fw *args:
    just build_cli_app
    cargo build --release
    ./../flasher/target/release/flasher {{args}} flash --application ../target/thumbv7em-none-eabihf/release/firmware --compress -l

ident *args:
    just build_cli_app
    ./flasher/target/release/flasher {{args}} ident