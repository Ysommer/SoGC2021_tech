from cgshop2021_pyutils import InstanceDatabase


def load_data_base():
    return InstanceDatabase("../instances/instances-zip/instances.zip")


def update_table_of_content():
    idb = load_data_base()
    counter = 0
    f = open("TableOfContents.txt", "w")
    for i in idb:
        print(counter, ":", i.name, file=f)
        counter += 1
    f.close()


def load_all_instances():
    idb = load_data_base()
    instances = []

    for i in idb:
        instances.append(i)

    return instances


def load_instance(index):
    return load_all_instances()[index]