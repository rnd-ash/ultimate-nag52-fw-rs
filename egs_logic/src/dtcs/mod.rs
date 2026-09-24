pub mod debouncer;

pub enum DebounceType {
    Event(debouncer::CounterDebouncer),
    Time(debouncer::TimerDebouncer)
}