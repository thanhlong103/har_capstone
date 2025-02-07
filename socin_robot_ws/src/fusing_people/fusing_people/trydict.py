import numpy as np

people = {}


def add_person(
    person_id,
    pos_x,
    pos_y,
    vel_x=0.0,
    vel_y=0.0,
    orientation_x=0.0,
    orientation_y=0.0,
    orientation_z=0.0,
    orientation_w=0.0,
):
    people[person_id] = {
        "filtered_state_means": np.array([0.0, 0.0, 0.0, 0.0]),
        "pos_x": pos_x,
        "pos_y": pos_y,
        "vel_x": vel_x,
        "vel_y": vel_y,
        "orientation_x": orientation_x,
        "orientation_y": orientation_y,
        "orientation_z": orientation_z,
        "orientation_w": orientation_w,
        "pos_x_laser": 0.0,
        "pos_y_laser": 0.0,
        "pos_x_vision": 0.0,
        "pos_y_vision": 0.0,
    }


def update_person(self, person_id, key, value):
    if person_id in self.people and key in self.people[person_id]:
        self.people[person_id][key] = value
