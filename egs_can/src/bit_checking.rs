use crate::CanResult;
use num_traits::Bounded;

pub trait SafeCanValue<T> {
    /// Checks if CAN Value is in a safe valid state by discounting
    /// values where all bits are 1's (Error/Uninit state)
    fn check_valid(self) -> CanResult<T>;

    /// Helper function for `x.check_valid().map(|x| f(x))`
    fn with_valid<R, F: FnOnce(T) -> R>(self, f: F) -> CanResult<R>
    where
        Self: Sized,
    {
        self.check_valid().map(|x| f(x))
    }
}

impl<T> SafeCanValue<T> for CanResult<T>
where
    T: Copy + Eq + Bounded,
{
    fn check_valid(self) -> CanResult<T> {
        self.and_then(|v| {
            if v == T::max_value() {
                Err(crate::CanError::SignalInvalid)
            } else {
                Ok(v)
            }
        })
    }
}

impl<T> SafeCanValue<T> for T
where
    T: Copy + Eq + Bounded,
{
    fn check_valid(self) -> CanResult<T> {
        if self == T::max_value() {
            Err(crate::CanError::SignalInvalid)
        } else {
            Ok(self)
        }
    }
}
