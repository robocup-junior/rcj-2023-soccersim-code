def get_direction(ball_vector: list) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_vector (list of floats): Current vector of the ball with respect
            to the robot.

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    #if -0.13 <= ball_vector[1] <= 0.13:
       # return 0
    #return -1 if ball_vector[1] < 0 else 1

    if -0.13 <= ball_vector[1] <= 0.13   :
        return 0 
    if ball_vector[1] ==0.7:
        return 0
    elif ball_vector[1] < 0 :
        return -1
    else:
        return 1


    if -0.6 <= ball_vector[1] <= 0.6:
        return 2
    else :
        return 3


    if ball_vector[1]==0.7:
        return 6
    else:
        return 7



    





    
