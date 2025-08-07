import carla
import glob
import os
import sys
import time
import logging
from numpy import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def get_pedestrian_blueprints(world, filter='walker.pedestrian.*', generation='2'):
    bps = world.get_blueprint_library().filter(filter)
    if generation.lower() == "all":
        return bps
    try:
        int_gen = int(generation)
        return [x for x in bps if int(x.get_attribute('generation')) == int_gen]
    except:
        print("Invalid")
        return []

def main():
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    seed = 42
    random.seed(seed)
    world.set_pedestrians_seed(seed)
    walkers_list = []
    all_id = []

    try:
        #no. of walkers, running % and crossing %
        number_of_walkers = 100
        percentage_running = 0.0
        percentage_crossing = 1.0

        blueprintsWalkers = get_pedestrian_blueprints(world)

        #Generation of random spawn points
        spawn_points = []
        for _ in range(number_of_walkers):
            loc = world.get_random_location_from_navigation()
            if loc:
                spawn_points.append(carla.Transform(loc))

        #Spawn Walkers
        walker_speeds = []
        walker_batch = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            if walker_bp.has_attribute('speed'):
                if random.random() > percentage_running:
                    walker_speeds.append(walker_bp.get_attribute('speed').recommended_values[1])  # walk
                else:
                    walker_speeds.append(walker_bp.get_attribute('speed').recommended_values[2])  # run
            else:
                walker_speeds.append(0.0)
            walker_batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

        results = client.apply_batch_sync(walker_batch, True)
        walker_speed2 = []
        for i, result in enumerate(results):
            if result.error:
                logging.error(result.error)
            else:
                walkers_list.append({"id": result.actor_id})
                walker_speed2.append(walker_speeds[i])
        walker_speeds = walker_speed2

        #Spawning controllers
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        controller_batch = []
        for walker in walkers_list:
            controller_batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walker["id"]))
        results = client.apply_batch_sync(controller_batch, True)
        for i, result in enumerate(results):
            if result.error:
                logging.error(result.error)
            else:
                walkers_list[i]["con"] = result.actor_id

        for i in range(len(walkers_list)):
            all_id.extend([walkers_list[i]["con"], walkers_list[i]["id"]])

        all_actors = world.get_actors(all_id)

        #Controller control at random points
        world.set_pedestrians_cross_factor(percentage_crossing)
        for i in range(0, len(all_id), 2):
            controller = all_actors[i]
            walker = all_actors[i + 1]
            controller.start()
            controller.go_to_location(world.get_random_location_from_navigation())
            controller.set_max_speed(float(walker_speeds[int(i/2)]))

        print(f"Spawned {len(walkers_list)} pedestrians. Press Ctrl+C to exit.")

        while True:
            world.wait_for_tick()

    finally:
        print('\nCleaning')
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
        time.sleep(0.5)
        print("Cleanup complete.")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass




