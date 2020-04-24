import argparse


def main():
    parser = argparse.ArgumentParser()
    # Load file and normalize according first coordinate
    f_rel = open("waypoints/zalazone_smart_city.rel.csv", 'w')
    with open("waypoints/zalazone_smart_city.csv") as f:
        header = f.readline()
        f_rel.write(header)
        first_line = f.readline().strip().split(',')
        offset_x = float(first_line[0])
        offset_y = float(first_line[1])
        f_rel.write("{0},{1},{2},{3},{4},{5}\n".format(0.0, 0.0, first_line[2], first_line[3], first_line[4], first_line[5]))
        for raw_line in f:
            line = raw_line.strip().split(',')
            f_rel.write("{0},{1},{2},{3},{4},{5}\n".format(
                float(line[0]) - offset_x, 
                float(line[1]) - offset_y, 
                line[2], line[3], line[4], line[5]))




if __name__=="__main__":
    main()