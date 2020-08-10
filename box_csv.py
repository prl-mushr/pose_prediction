import csv


with open('car_box.csv', 'w', newline='') as csvfile:
    fieldnames = ['x_min', 'x_max', 'y_min','y_max']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    writer.writerow({'x_min': xmin, 'x_max': xmax,'y_min': ymin, 'y_max': ymax})
