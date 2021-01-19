from time import time


def load_script(file_name):
    try:
        file = open(file_name, 'r').readlines()
    except:
        return None
    x = file.index('run:\n')
    file = file[x + 1:]
    file = [s.replace('\n', '').replace(' ', '') for s in file]

    instruction_list = [[s[0], False, [float(i) for i in s[2:-1].split(',')] if s[0] == 'g' else float(s[1:])] for s
                        in file]
    return 0, 0, instruction_list


class Script:
    queue = 0
    instruction = None
    instruction_list = None

    def __init__(self, file_name):
        self.queue, self.instruction, self.instruction_list = load_script(file_name)


script = Script('run_script.txt')

print(script)

while True:
    if script.instruction == 'w':
        if instruction[1] == False:
            start = time()
            instruction[1] = True
        elif time() - start > instruction[2]:
            instruction = instruction_list[(queue := queue + 1)]
    if instruction[0] == 'g':
        instruction[1] = True
        goal_theta = instruction[2]
        print(goal_theta)
        instruction = instruction_list[(queue := queue + 1)]
    if instruction[0] == 'e':
        pass
