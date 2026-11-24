# Runs before every build (platformio.ini: extra_scripts).
# Stops with a clear message when program_name does not match a file, instead of the
# linker error "undefined reference to setup / loop".
import os

Import("env")

name = env.GetProjectConfig().get("project_options", "program_name").strip()
project = env.subst("$PROJECT_DIR")
found = [p for p in (os.path.join(project, "src", name + ".cpp"),
                     os.path.join(project, "src", "test", name + ".cpp")) if os.path.isfile(p)]

if len(found) != 1:
    print("\n*** program_name = '%s': %s" % (name, "no such file in src/ or src/test/" if not found
                                              else "found in both src/ and src/test/"))
    print("*** Check the spelling in platformio.ini, or update your local repo (git pull).\n")
    env.Exit(1)
else:
    print("Building program: " + os.path.relpath(found[0], project))
