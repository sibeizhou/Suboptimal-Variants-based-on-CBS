import random
import string

list2 = [20,30,40,50]

fileNumber = 1
for k in list2:
    filename = 'largeTest_'
    filename2 = ".txt"
    file = open(filename+ str(fileNumber) + filename2, "w")
    row = 20
    col = 20
    rate = 0.2

    # for i in range(row * col):
    #     a = random.randrange(0, 9, 1)
    #     if a == 1:
    #         list1.append(1)
    #     else:
    #         list1.append(0)

    space_list = random.sample(range(0, row * col - 1), int(row * col * (1 - rate)))
    start_point_list = random.sample(space_list, k)
    end_point_list = random.sample(space_list, k)

    file.write(str(row) + " " + str(row) + "\n")
    index = 0
    for i in range(row):
        for j in range(col):
            if index in space_list:
                file.write(". ")
            else:
                file.write("@ ")
            index += 1
        file.write("\n")

    file.write(str(k) + "\n")

    for agent_index in range(k):
        str_agent = str(int(start_point_list[agent_index]/col)) + " " \
                    + str(start_point_list[agent_index]%col) + " " \
                    + str(int(end_point_list[agent_index]/col)) + " " \
                    + str(end_point_list[agent_index]%col)
        file.write(str_agent)
        file.write("\n")

    fileNumber = fileNumber + 1