""" Create Disassembly from elf file as post build action """
import platform

Import("env", "projenv")

dump_tool = None
elf_target = "$BUILD_DIR/${PROGNAME}.elf"


if env["PIOPLATFORM"] == "espressif32":
    dump_tool = "xtensa-esp32-elf-objdump"

elif env["PIOPLATFORM"] == "atmelavr":
    dump_tool = "avr-objdump"

elif env["PIOPLATFORM"] == "native":
    dump_tool = "objdump"

    if platform.system() != "Windows":
        elf_target = "$BUILD_DIR/${PROGNAME}"

if dump_tool is not None:
    env.AddPostAction(
        elf_target,
        env.VerboseAction(" ".join([
            dump_tool, "-h", "-l", "-S", "-C", elf_target, ">", "$BUILD_DIR/${PROGNAME}.dasm"
        ]),
        "Creating assembler listing file $BUILD_DIR/${PROGNAME}.dasm")
    )