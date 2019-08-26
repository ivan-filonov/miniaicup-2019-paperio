#!/bin/bash
FILES=$(find . \( -iname "*.cpp" -or -iname "*.h*" \))
for f in ${FILES}
do
  sed -i "s/AICUP_ENVIRONMENT 0/AICUP_ENVIRONMENT 1/" ${f}
done
zip -9 solution_cpp17 ${FILES}
for f in ${FILES}
do
  sed -i "s/AICUP_ENVIRONMENT 1/AICUP_ENVIRONMENT 0/" ${f}
done
