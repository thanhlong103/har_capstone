class FakeCoordinate():
    def __init__(self, point_x, point_y):
        self.x = point_x
        self.y = point_y
class FakePoint():
    def __init__(self, point_x, point_y):
        self.point = FakeCoordinate(point_x, point_y)

class FakePerson():
    def __init__(self, point_x, point_y, vel_x, vel_y):
        self.pose = FakePoint(point_x, point_y)
        self.velocity = FakeCoordinate(vel_x, vel_y)

def simulate_person_array(num_person):
    person_array = []
    simulated_person = [[1, 0, 0, 0], [2, -1.5, 0, 0], [1, -0.5, 0, 0], [0.5, 3, 0, 0]]
    for i in range(num_person):
        point_x = simulated_person[i][0]
        point_y = simulated_person[i][1]
        vel_x = simulated_person[i][2]
        vel_x = simulated_person[i][3]
        person_array.append(FakePerson(point_x, point_y, vel_x, vel_y))
    
    return person_array

def main():
    person_A = FakePerson(1,1,0,0)
    print(person_A.pose.point.x, person_A.pose.point.y)
    print(person_A.velocity.y, person_A.velocity.y)

if __name__ == '__main__':
    main()