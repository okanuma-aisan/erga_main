#!/usr/bin/env python
import pathlib
import argparse
import os

def make_file(path):
    with open(path, 'w') as fp:
        fp.write()
        fp.flush()

def replace_in_file(path, replacements):
    fp_in = open(path, 'r')
    data_in = fp_in.read()
    fp_in.close()

    for key in replacements.keys():
        src = key
        dst = replacements[key]
        data_in = data_in.replace(src, dst)
    
    fp_out = open(path, 'w')
    fp_out.write(data_in)
    fp_out.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("package_name")
    parser.add_argument("--license", default="GPLv3", required=False)
    parser.add_argument("--description", default="A ROS2 python package.", required=False)
    parser.add_argument("--version", default="0.0.0", required=False)
    args = parser.parse_args()

    python_package_dir = pathlib.Path("./PACKAGE_NAME")
    python_package_dir.rename(args.package_name)

    resource_file = pathlib.Path(f"./resource/{args.package_name}")
    resource_file.touch(exist_ok=True)

    replacement_files = [
        "package.xml",
        "setup.cfg",
        "setup.py",
        f"{args.package_name}/main.py"
    ]
    replacements = {
        "!!PACKAGE_NAME!!": f"{args.package_name}",
        "!!PACKAGE_DESCRIPTION!!": f"{args.description}",
        "!!PACKAGE_VERSION!!": f"{args.version}",
        "!!PACKAGE_LICENSE!!": f"{args.license}"
    }
    for file_path in replacement_files:
        replace_in_file(file_path, replacements)


