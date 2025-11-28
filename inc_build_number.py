#!/usr/bin/env python3
import os
import re 
# Import("env")


def increment_ver(version):
    version = str(int(version) + 1)
    return version

def update_version_strings(file_path):
    version_regex = re.compile(r"^(#define\sBUILD\s)(\d+)", re.MULTILINE)
    with open(file_path, "r+") as f:
    
        content = f.read()
        f.seek(0)
        f.write(
            re.sub(
                version_regex,
                lambda match: "{}{}".format(match.group(1), increment_ver(match.group(2))),
                content,
            )
        )
        f.truncate()

'''
def pre_program_action(source, target, env):
    print("Program will be built ...")
    program_path = target[0].get_abspath()
    print("Program path", program_path)
    update_version_strings("include/NMEA_MUX.h")
'''

#env.AddPreAction("$PROGPATH", pre_program_action)
# This below is called before compilation, kinda workarount
update_version_strings("include/NMEA_MUX.h")