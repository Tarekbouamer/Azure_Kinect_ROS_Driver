import ast

import ConfigParser
from os import path, listdir

_CONVERTERS = {
    "struct": ast.literal_eval
}

_DEFAULTS_DIR = path.abspath(path.join(path.split(__file__)[0], "default"))
DEFAULTS = dict()
for file in listdir(_DEFAULTS_DIR):
    name, ext = path.splitext(file)
    if ext == ".ini":
        DEFAULTS[name] = path.join(_DEFAULTS_DIR, file)


def load_config(config_file, defaults_file):
    par = ConfigParser.ConfigParser()
    par.read(defaults_file)
    return par