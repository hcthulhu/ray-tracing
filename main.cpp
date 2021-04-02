#include "ray_tracing.h"
#include "mpi.h"

int main(int argc, char* argv[]) {
	const int height = 1000;
	const int width = 1400;
    unsigned char pixels[height * width * 3];
    bzero(pixels, height * width * 3);
	const float viewingAngle = 3.14 / 2;
	float x1, y1;
	double time;

	int size, rank;
	MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &size);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	std::vector<Triangle> triangles;
	readTrianglesFromFile("../model.xyz", triangles, 10000);

	if (rank == 0) {
		std::cout << "Cluster size: " << size << std::endl;
		std::cout << "Triangles amount: " << triangles.size() << std::endl;
		time = MPI_Wtime();
	}

	Vec3f dir;
	Vec3f origin(0, 0, 0);
	Vec3f dir_0(1, 0, 0);
	dir_0.normalize();

	PointLight pLight1(Vec3f (0, -10, 0), Vec3f (0, 0, 1));
    PointLight pLight2(Vec3f (0, 0, 0), Vec3f (0.5, 0.5, 0.5));
    PointLight pLight3(Vec3f (0, 10, 0), Vec3f (1, 0, 0));
    std::vector<PointLight> pLights = {pLight1, pLight2, pLight3};

	Vec3f color;
	std::vector<Vec3f> basis = find_basis(dir_0);
	Vec3f x_basis = basis[0];
	Vec3f y_basis = basis[1];

    if (rank == 0) {
        std::cout << "dir_0 = { " << dir_0 << "}" << std::endl;
        std::cout << "plane basis = { " << x_basis << "}, {" << y_basis << "}" << std::endl;
    }

    for (size_t j = height / size * rank; j < height / size * (rank + 1); j++) {
        for (size_t i = 0; i < width; i++) {
            x1 = (2 * (i + 0.5) / (float)width - 1.) * tan(viewingAngle / 2.) * width / (float)height;
            y1 = -(2 * (j + 0.5) / (float)height - 1.) * tan(viewingAngle / 2.);
            dir = (x_basis * x1 + y_basis * y1 + dir_0).normalize();

            color = castRayColor(origin, dir, pLights, triangles, 0) * LIGHT_BASE;

            color.x = color.x > 255 ? 255 : color.x;
            color.y = color.y > 255 ? 255 : color.y;
            color.z = color.z > 255 ? 255 : color.z;

            pixels[3 * (i + j * width)] = (char)color.x;
            pixels[3 * (i + j * width) + 1] = (char)color.y;
            pixels[3 * (i + j * width) + 2] = (char)color.z;

        }
        std::cout << j << std::endl;
    }

    MPI_Gather(&pixels[height / size * rank * width * 3], height * width * 3 / size, MPI_CHAR, pixels, height * width * 3 / size, MPI_CHAR, 0, MPI_COMM_WORLD);

	if (rank == 0) {
		std::cout << "Time: " << MPI_Wtime() - time << std::endl;
		char pic[] = "../result.jpg";
		saveImage(pic, width, height, 3, pixels);
	}

	MPI_Finalize();

	return 0;
}
