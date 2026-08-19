# Unit art files

Each file here defines one function, `print_unit_art_<key>()`, where `<key>`
matches that unit type's entry in `UNIT_KEY` in `include/units/registry.sh`.
`include/menus/unit_art.sh` sources every `*.sh` file in this directory at
startup and dispatches to the right function via `print_unit_art
"$installation_type"`.

The function body is plain bash -- normally one or two `printf` calls that
write the ASCII/ANSI-art banner and an attribution line to stdout, e.g.:

```sh
print_unit_art_MyUnit() {
  printf "...banner...\n"
  printf "MyUnit by Someone\n"
}
```

A single `printf "placeholder art\n"` is enough to get a new unit type
working; polish the banner later.
