from inputs import *
import sys, os


if __name__ == '__main__':
    if "-help" in sys.argv: print(
            "| command              | args      | description                       |",
            "|----------------------|-----------|-----------------------------------|",
            "| [-controller-list]   |           | list all available controllers    |",
            "| [-controller]        | %s        | name of the controller            |",
            sep="\n"
        ); exit(0)
    if "-controller-list" in sys.argv: print(*devices, sep="\n"); exit(0)

    # https://pypi.org/project/aioserial/#quick-start