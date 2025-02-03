def create_env(env_id, viz=False):
    obs_points = []
    center_points = []

    #sim env 1
    if env_id == 1:
        corner_length = 30
        #corner top left
        for i in range(corner_length):
            obs_points.append([-3 + 0.04*i, 2])
            obs_points.append([-3, 2 - 0.04 * i])
        center_points.append([-3 + 0.04*(corner_length//2), 2, 0.04 * (corner_length//2) + 0.02, 0.02])
        center_points.append([-3, 2 - 0.04*(corner_length//2), 0.02, 0.04 * (corner_length//2) + 0.02])

        #corner bottom left
        for i in range(corner_length):
            obs_points.append([-3 + 0.04*i, -2])
            obs_points.append([-3, -2 + 0.04 * i])
        center_points.append([-3 + 0.04*(corner_length//2), -2, 0.04 * (corner_length//2) + 0.02, 0.02])
        center_points.append([-3, -2 + 0.04*(corner_length//2), 0.02, 0.04 * (corner_length//2) + 0.02])

        #corner top right
        for i in range(corner_length):
            obs_points.append([3 - 0.04*i, 2])
            obs_points.append([3, 2 - 0.04 * i])
        center_points.append([3 - 0.04*(corner_length//2), 2, 0.04 * (corner_length//2) + 0.02, 0.02])
        center_points.append([3, 2 - 0.04*(corner_length//2), 0.02, 0.04 * (corner_length//2) + 0.02])


        #corner bottom right
        for i in range(corner_length):
            obs_points.append([3 - 0.04*i, -2])
            obs_points.append([3, -2 + 0.04 * i])
        center_points.append([3 - 0.04*(corner_length//2), -2, 0.04 * (corner_length//2) + 0.02, 0.02])
        center_points.append([3, -2 + 0.04*(corner_length//2), 0.02, 0.04 * (corner_length//2) + 0.02])


        #middle obs
        middle_length = 50
        for i in range(middle_length):
            obs_points.append([-1 + 0.04 * i, -1])
        center_points.append([-1 + 0.04*(middle_length//2), -1, 0.04 * (middle_length//2) + 0.02, 0.02])

        for i in range(middle_length):
            obs_points.append([-1, -1 + 0.04 * i])
            obs_points.append([1, -1 + 0.04 * i])
        center_points.append([-1, -1 + 0.04*(middle_length//2), 0.02, 0.04 * (middle_length//2) + 0.02])
        center_points.append([1, -1 + 0.04*(middle_length//2), 0.02, 0.04 * (middle_length//2) + 0.02])

    #sim env 2
    elif env_id == 2:
        corner_length = 40

        for i in range(corner_length):
            obs_points.append([-2, 0.04 * i])
            obs_points.append([-2, -0.04 * i])
            obs_points.append([2, 0.04 * i])
            obs_points.append([2, -0.04 * i])
        center_points.append([-2, 0, 0.02, 0.04 * (corner_length)])
        center_points.append([2, 0, 0.02, 0.04 * (corner_length)])

        #middle narrow opening
        open_length = 100
        for i in range(open_length):
            obs_points.append([0, 4.96 - 0.04 * i])
            obs_points.append([0, -4.96 + 0.04 * i])
        center_points.append([0, 4.96 - 0.04 * (open_length//2), 0.02, 0.04 * (open_length//2) + 0.02])
        center_points.append([0, -4.96 + 0.04 * (open_length//2), 0.02, 0.04 * (open_length//2) + 0.02])

    #sim env 3
    elif env_id == 3:
        corner_length = 100
        for i in range(corner_length):
            obs_points.append([-3, 2.96 - 0.04 * i])
            obs_points.append([3, -2.96 + 0.04 * i])
        center_points.append([-3, 2.96 - 0.04 * (corner_length//2), 0.02, 0.04 * (corner_length)//2 + 0.02])
        center_points.append([3, -2.96 + 0.04 * (corner_length//2), 0.02, 0.04 * (corner_length)//2 + 0.04])

        middle_length = 30
        for i in range(middle_length):
            obs_points.append([-2, -1 + 0.04 * i])
            obs_points.append([-2, -1 - 0.04 * i])
            obs_points.append([-2 + 0.04 * i, -1 + middle_length * 0.04])
            obs_points.append([-2 + 0.04 * i, -1 - middle_length * 0.04])

            obs_points.append([2, 1 + 0.04 * i])
            obs_points.append([2, 1 - 0.04 * i])
            obs_points.append([2 - 0.04 * i, 1 + middle_length * 0.04])
            obs_points.append([2 - 0.04 * i, 1 - middle_length * 0.04])
        center_points.append([-2, -1, 0.02, 0.04*middle_length + 0.02])
        center_points.append([-2 + 0.04 * (middle_length//2), -1 + 0.04*(middle_length), 0.04*(middle_length//2), 0.02])
        center_points.append([-2 + 0.04 * (middle_length//2), -1 - 0.04*(middle_length), 0.04*(middle_length//2), 0.02])

        center_points.append([2, 1, 0.02, 0.04*middle_length + 0.02])
        center_points.append([2 - 0.04 * (middle_length//2), 1 + 0.04*(middle_length), 0.04*(middle_length//2), 0.02])
        center_points.append([2 - 0.04 * (middle_length//2), 1 - 0.04*(middle_length), 0.04*(middle_length//2), 0.02])

    #sim env 4
    elif env_id == 4:
        x_length = 20
        for i in range(x_length * 2):
            obs_points.append([-2, i * 0.04])
            obs_points.append([-2, -i * 0.04])
            obs_points.append([2, i * 0.04])
            obs_points.append([2, -i * 0.04])
        center_points.append([-2, 0, 0.02, 0.04 * x_length * 2 + 0.02])
        center_points.append([2, 0, 0.02, 0.04 * x_length * 2 + 0.02])


        for i in range(x_length):
            obs_points.append([-2 - i * 0.04, x_length * 0.04 * 2])
            obs_points.append([-2 - i * 0.04, -x_length * 0.04 * 2])
            obs_points.append([2 + i * 0.04, x_length * 0.04 * 2])
            obs_points.append([2 + i * 0.04, -x_length * 0.04 * 2])
        center_points.append([-2 - 0.04*(x_length//2), x_length * 0.04 * 2, 0.04*(x_length//2), 0.02])
        center_points.append([-2 - 0.04*(x_length//2), -x_length * 0.04 * 2, 0.04*(x_length//2), 0.02])
        center_points.append([2 + 0.04*(x_length//2), x_length * 0.04 * 2, 0.04*(x_length//2), 0.02])
        center_points.append([2 + 0.04*(x_length//2), -x_length * 0.04 * 2, 0.04*(x_length//2), 0.02])


        y_length = 10
        for i in range(y_length * 2):
            obs_points.append([i * 0.04, -1])
            obs_points.append([-i * 0.04, -1])
            obs_points.append([i * 0.04, 1])
            obs_points.append([-i * 0.04, 1])
        center_points.append([0, -1, 0.04 * y_length * 2 + 0.02, 0.02])
        center_points.append([0, 1, 0.04 * y_length * 2 + 0.02, 0.02])

        old_y_length = y_length
        y_length += 7
        for i in range(y_length):
            obs_points.append([0.04 * old_y_length * 2, -1 - 0.04 * i])
            obs_points.append([-0.04 * old_y_length * 2, -1 - 0.04 * i])
            obs_points.append([0.04 * old_y_length * 2, 1 + 0.04 * i])
            obs_points.append([-0.04 * old_y_length * 2, 1 + 0.04 * i])
        center_points.append([0.04 * 2 * old_y_length, -1 - 0.04*(y_length//2), 0.02, 0.04*(y_length//2)])
        center_points.append([-0.04 * 2 * old_y_length, -1 - 0.04*(y_length//2), 0.02, 0.04*(y_length//2)])
        center_points.append([0.04 * 2 * old_y_length, 1 + 0.04*(y_length//2), 0.02, 0.04*(y_length//2)])
        center_points.append([-0.04 * 2 * old_y_length, 1 + 0.04*(y_length//2), 0.02, 0.04*(y_length//2)])
    
    return center_points, obs_points

def get_config(case_id):
    if case_id == 0:
        #test case1 - env2
        space = [[-5, 5], [-5, 5]]  # X and Y ranges
        env_id = 2
        start_config = [[-3.0, -1], [-3.0, 1]]
        goal_config = [[3.0, 1], [3.0, -1]]

    if case_id == 1:
        #test case1 - env2
        space = [[-5, 5], [-5, 5]]  # X and Y ranges
        env_id = 2
        start_config = [[-3.0, -1], [3.0, -1], [-3.0, 1]]
        goal_config = [[3.0, -1], [3.0, 1], [-3.0, -1]]

    elif case_id == 2:
        #test case2 - env2
        space = [[-5, 5], [-5, 5]]  # X and Y ranges
        env_id = 2
        start_config = [[-3.0, -1], [3.0, -1], [-3.0, 1], [3.0, 1]]
        goal_config = [[3.0, -1], [3.0, 1], [-3.0, -1], [-3.0, 1]]

    elif case_id == 3:
        #test case3 - env2
        space = [[-5, 5], [-5, 5]]  # X and Y ranges
        env_id = 2
        start_config = [[-3.0, -1], [3.0, -1], [-3.0, 1], [3.0, 1]]
        goal_config = [[3.0, -1], [-3.0, -1], [3.0, 1], [-3.0, 1]]

    elif case_id == 4:
        # test case5 3 way crossing
        space = [[-5, 5], [-5, 5]]  # X and Y ranges
        env_id = 0
        start_config = [[-3.0, -3.0], [0.0, -3.0], [3.0, -3.0]]
        goal_config = [[3.0, 3.0], [0.0, 3.0], [-3.0, 3.0]]

    elif case_id == 5:
        # test case6 4 way swap pos
        space = [[-5, 5], [-5, 5]]  # X and Y ranges
        env_id = 0
        start_config = [[-3.0, 0], [3.0, 0], [0, 3], [0, -3]]
        goal_config = [[3.0, 0], [-3.0, 0], [0, -3], [0, 3]]

    elif case_id == 6:
        # test case6 8 way swap pos
        space = [[-5, 5], [-5, 5]]  # X and Y ranges
        env_id = 0
        start_config = [[-3.0, 0], [3.0, 0], [0, 3], [0, -3], [-2, -2], [2, 2], [-2, 2], [2, -2]]
        goal_config = [[3.0, 0], [-3.0, 0], [0, -3], [0, 3], [2, 2], [-2, -2], [2, -2], [-2, 2]]

    return space, env_id, start_config, goal_config
