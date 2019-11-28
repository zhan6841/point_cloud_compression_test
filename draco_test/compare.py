import csv

def compare(input, output):
	input.sort()
	output.sort()
	# print(input)
	# print(output)
	print("# of input:", len(input))
	print("# of output:", len(output))
	if(len(input) != len(output)):
		print('The lengths of input and output are not equal')
		exit(0)
	diff = []
	count = 0
	for i in range(0, len(input)):
		diff.append(output[i] - input[i])
		if(diff[-1] < 0.001 and diff[-1] > -0.001):
		# if(diff[-1] > 0.01):
			count += 1
		# if(diff[-1] == 0.0):
			# print(str(input[i]) + ' ' + str(output[i]))
	print("equal:", diff.count(0.0))
	print("(-0.001, 0.001):", count)
	# print(diff)


def load(filename):
	x = []
	y = []
	z = []
	intensity = []
	with open(filename, 'r') as f:
		f_csv = csv.reader(f, delimiter=' ')
		for row in f_csv:
			x.append(float(format(float(row[0]), '.3f')))
			y.append(float(format(float(row[1]), '.3f')))
			z.append(float(format(float(row[2]), '.3f')))
			intensity.append(float(row[3]))
	return x, y, z, intensity

if __name__ == '__main__':
	in_x, in_y, in_z, in_intensity = load('in.xyz')
	out_x, out_y, out_z, out_intensity = load('out.xyz')
	print('compare x:')
	compare(in_x, out_x)
	print('compare y:')
	compare(in_y, out_y)
	print('compare z:')
	compare(in_z, out_z)
	print('compare intensity:')
	compare(in_intensity, out_intensity)