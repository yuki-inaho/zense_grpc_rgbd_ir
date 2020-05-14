#!/bin/bash

pip install -r requirements.txt --user

cd pico_zense_cython_wrapper
pip install -r requirements.txt --user
python setup.py install --user
cd ..

mkdir -p build && cd build 
cmake .. && make -j && cd ..
cd /app/scripts && python gen_protobuf_codes.py