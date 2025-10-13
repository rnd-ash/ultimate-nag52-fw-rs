use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};


#[proc_macro_derive(CalibrationStructSig)]
pub fn create_struct_signature(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = input.ident;

    // Only for struct attributes
    if let  syn::Data::Struct(field_struct) = input.data {
        let mut descriptor = String::new();
        // Add every field name and type to a string
        for field in field_struct.fields {
            descriptor.push_str(&format!("{:?}{:?}", field.ident, field.ty));
        }
        let mut hash: u32 = 0;
        for (idx, byte) in descriptor.as_bytes().iter().enumerate() {
            hash = hash
                .wrapping_add(*byte as u32)
                .wrapping_add(idx as u32)
        }
        // Generate a SIGNATURE constant for the struct
        quote! {
            impl #name {
                pub const CAL_SIGNATURE: u32 = #hash;
            }
        }.into()
    } else  {
        syn::Error::new_spanned(name, "CalibrationStructSig can only be applied to structs")
            .to_compile_error()
            .into()
    }

}
