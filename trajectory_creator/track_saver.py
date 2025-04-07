from track_trajectory_generator import generate_loop_track
import csv


def save_points_to_csv(points, filename):
    """Zapisuje listę punktów (x, y) do pliku CSV"""
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y"])  # nagłówek
        for x, y in points:
            writer.writerow([x, y])

def main():
    center_line, left_edge, right_edge = generate_loop_track()

    save_points_to_csv(center_line, "./track_data/centerline.csv")
    save_points_to_csv(left_edge, "./track_data/left_edge.csv")
    save_points_to_csv(right_edge, "./track_data/right_edge.csv")

    print("Zapisano trajektorię do: centerline.csv, left_edge.csv, right_edge.csv")

if __name__ == "__main__":
    main()
