"""
Pre-build script: renombra library.properties.txt → library.properties
en todas las librerías descargadas por PlatformIO, para evitar el error
MissingPackageManifestError.
"""
import os
Import("env")  # noqa: F821  — inyectado por PlatformIO SCons

libdeps_dir = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env.subst("$PIOENV"))

if os.path.isdir(libdeps_dir):
    for lib in os.listdir(libdeps_dir):
        bad = os.path.join(libdeps_dir, lib, "library.properties.txt")
        good = os.path.join(libdeps_dir, lib, "library.properties")
        if os.path.isfile(bad) and not os.path.isfile(good):
            os.rename(bad, good)
            print(f"  [fix_lib_manifests] {lib}: library.properties.txt → library.properties")
