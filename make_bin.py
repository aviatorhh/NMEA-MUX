#!/usr/bin/env python3
# import os

Import("env", "projenv")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "binary", "$TARGET",
        "'$BUILD_DIR/${PROGNAME}.bin'"
    ]), "Building '$BUILD_DIR/${PROGNAME}.bin'")
)