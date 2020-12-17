from cgshop2021_pyutils import InstanceDatabase


def load_data_base():
    return InstanceDatabase("../instances/instances-zip/instances.zip")


def update_table_of_content():
    f = open("TableOfContents.txt", "w")
    instances = load_all_instances()

    for i in range(len(instances)):
        print(i, ":", instances[i].name, file=f)
    f.close()


def load_all_instances():
    idb = load_data_base()
    instances = []

    for i in idb:
        instances.append(i)
    instances.sort(key = lambda x: x.name)
    return instances


def load_instance(index):
    return load_all_instances()[index]

update_table_of_content()