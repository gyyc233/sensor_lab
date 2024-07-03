find . -regextype egrep -regex ".*\.(c|cc|cpp|hpp|h|hh)$" -not -path '*/install/*' \
  -not -path '*/build/*' -not -path '*/log/*' -not -path '*/deps/*'| xargs clang-format-7 -i
  