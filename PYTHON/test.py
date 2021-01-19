from dataclasses import dataclass


@dataclass
class Intruction:
    command: chr
    data: None


p = Intruction('w', [1, 1, 1, 1])

print(p)
