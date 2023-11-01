#! /bin/bash

echo "Formatting code..."
find metrics filters grpc registration mapping scan_matching \( -iname *.h -o -iname *.cpp -o -iname *.md \) -and -not -iname *duna_exports.h | xargs clang-format -i --verbose
