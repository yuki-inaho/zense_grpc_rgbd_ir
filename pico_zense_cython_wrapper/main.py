#!/usr/bin/env python
# -*- coding: utf-8 -*-
import toml
import click
from zense_pywrapper import PyPicoZenseManager
from collections import OrderedDict


@click.command()
@click.option('--out', '-o', default='./camera_parameter.toml')
def main(out):
    zense = PyPicoZenseManager(0)

    decoder = toml.TomlDecoder(_dict=OrderedDict)
    encoder = toml.TomlEncoder(_dict=OrderedDict)
    toml.TomlEncoder = encoder
    dict_toml = toml.load(open('template.toml'),
                          _dict=OrderedDict, decoder=decoder)

    dict_toml["Camera0"]["serial_no"] = zense.getSerialNumber()
    params = zense.getCameraParameter()

    intrinsic_elems = ["fx", "fy", "cx", "cy", \
                       "p1", "p2", "k1", "k2", \
                       "k3", "k4", "k5", "k6"]
    for i, _elem in enumerate(intrinsic_elems):
        dict_toml["Camera0_Factory"][_elem] = params[i]

    with open(out, "w") as f:
        toml.encoder.dump(dict_toml, f)
        print("generated")

    del zense


if __name__ == "__main__":
    main()
