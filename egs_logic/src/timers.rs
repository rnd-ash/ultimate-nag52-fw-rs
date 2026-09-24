#[macro_export]
macro_rules! create_timers_storage {
    ({ $($timer_name:ident),* }) => {
        pastey::paste! {
            #[repr(C)]
            #[derive(Copy, Clone)]
            /// Storage for all EGS Timer variables
            pub struct AllEgsTimers {
                $(
                    pub [<$timer_name:snake>]: egs_maths::egs_timers::EgsCountDownTimer,
                )*
            }

            impl AllEgsTimers {
                pub const fn new() -> Self {
                    Self {
                        $(
                            [<$timer_name:snake>]: egs_maths::egs_timers::EGS_TIMER_EMPTY,
                        )*
                    }
                }

                pub fn update(&mut self) {
                    $(
                        self.[<$timer_name:snake>].decrement();
                    )*
                }
            }
        }
    };
}
