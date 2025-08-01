#!/usr/bin/env python3

with open("src/index.html", "r") as f:
    html = f.read()

with open("src/index.h", "w") as f:
    f.write("// Auto-generated from index.html\n")
    f.write('const char INDEX_HTML[] PROGMEM = R"rawliteral(\n')
    f.write(html)
    f.write('\n)rawliteral";\n')
