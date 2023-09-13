import carla


client=carla.Client('localhost',2000)
world=client.get_world()
world = client.load_world('Town04')
