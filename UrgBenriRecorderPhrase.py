import math

FILE = "2023_03_01_00_33_03_275.ubh"
OUTFILE = 'output.txt'

# def CacheDirections():
#     offset = (math.pi * 2 / 1440) * 540;     
#     directions = []
#     for i in range(0, 1081):
#         a = (math.pi * 2 / 1440) * i + offset
#         directions.append([-math.cos(a), -math.sin(a)])
#     return directions

# Directions = CacheDirections()

with open(FILE, 'r') as fin:
    Lines = fin.readlines()
    with open(OUTFILE, 'w') as fout:
        for index, line in enumerate(Lines):
            if "[scan]" in line:
                # step_data = Lines[index + 1]
                # step_data = [f"{Directions[i][0] * int(data)}, {Directions[i][1] * int(data)}" for i, data in enumerate(step_data.split(';'))]
                # step_data = ";".join(step_data)
                # fout.write(step_data + "\n")
                fout.write(Lines[index + 1])


# with open(OUTFILE, 'r') as f:
#     Lines = f.readlines()
#     for line in Lines:
#         spliteData = line.split(';')
#         print(len(spliteData))
       
        