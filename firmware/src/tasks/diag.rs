use crate::{Mono, app};
use app::diag_task::LocalResources;
use atsamd_hal::{fugit::ExtU64, rtic_time::Monotonic};
use futures::FutureExt;

pub async fn diag_task(ctx: app::diag_task::Context<'_>) {
    let LocalResources {
        usb_isotp_thread,
        isotp_thread,
        diag_server,
        ..
    } = ctx.local;
    let mut is_usb: bool = false;
    loop {
        let deadline = Mono::now() + 20u64.millis();

        futures::select_biased! {
            buf = isotp_thread.read_payload().fuse() => {
                is_usb = false;
                let response = diag_server.process_cmd(
                    buf.payload(),
                    Mono::now().duration_since_epoch().to_millis(),
                ).await;
                let _ = isotp_thread.write_payload(&mut Mono, response).await;
            },
            buf = usb_isotp_thread.read().fuse() => {
                is_usb = true;
                let response = diag_server.process_cmd(
                    buf.payload(),
                    Mono::now().duration_since_epoch().to_millis(),
                ).await;
                let _ = usb_isotp_thread.write(response).await;
            },
            _ = Mono::delay_until(deadline).fuse() => {
                // Fallthrough so we update KWP server
            }
        }
        if let Some(tx) = diag_server
            .update(Mono::now().duration_since_epoch().to_millis())
            .await
        {
            match is_usb {
                true => {
                    let _ = usb_isotp_thread.write(tx).await;
                }
                false => {
                    let _ = isotp_thread.write_payload(&mut Mono, tx).await;
                }
            }
        }
    }
}
