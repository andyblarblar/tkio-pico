/// Checks if a enum is of a variant, returning bool.
macro_rules! is_of_var {
    ($val:ident, $var:path) => {
        match $val {
            $var{..} => true,
            _ => false
        }
    }
}