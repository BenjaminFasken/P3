from secrets import randbelow
from string import printable


def generate_key():
    a = ""
    for i in range(30):
        a += printable[randbelow(94)]

    print(a)


while True:
    generate_key()
    generate_key()
    generate_key()
    var = input("\nPress enter to generate new passwords:\n")
