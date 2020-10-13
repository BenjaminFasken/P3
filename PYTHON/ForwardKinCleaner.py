import string
s = """
Matrix(4, 4, [[(cos(theta__1)*cos(theta__2)*cos(theta__3) - cos(theta__1)*sin(theta__2)*sin(theta__3))*cos(theta__4) + (-cos(theta__1)*cos(theta__2)*sin(theta__3) - cos(theta__1)*sin(theta__2)*cos(theta__3))*sin(theta__4), -sin(theta__1), -(cos(theta__1)*cos(theta__2)*cos(theta__3) - cos(theta__1)*sin(theta__2)*sin(theta__3))*sin(theta__4) + (-cos(theta__1)*cos(theta__2)*sin(theta__3) - cos(theta__1)*sin(theta__2)*cos(theta__3))*cos(theta__4), ((cos(theta__1)*cos(theta__2)*cos(theta__3) - cos(theta__1)*sin(theta__2)*sin(theta__3))*cos(theta__4) + (-cos(theta__1)*cos(theta__2)*sin(theta__3) - cos(theta__1)*sin(theta__2)*cos(theta__3))*sin(theta__4))*d__4 + (cos(theta__1)*cos(theta__2)*cos(theta__3) - cos(theta__1)*sin(theta__2)*sin(theta__3))*d__3 + cos(theta__1)*cos(theta__2)*d__2], [(sin(theta__1)*cos(theta__2)*cos(theta__3) - sin(theta__1)*sin(theta__2)*sin(theta__3))*cos(theta__4) + (-sin(theta__1)*cos(theta__2)*sin(theta__3) - sin(theta__1)*sin(theta__2)*cos(theta__3))*sin(theta__4), cos(theta__1), -(sin(theta__1)*cos(theta__2)*cos(theta__3) - sin(theta__1)*sin(theta__2)*sin(theta__3))*sin(theta__4) + (-sin(theta__1)*cos(theta__2)*sin(theta__3) - sin(theta__1)*sin(theta__2)*cos(theta__3))*cos(theta__4), ((sin(theta__1)*cos(theta__2)*cos(theta__3) - sin(theta__1)*sin(theta__2)*sin(theta__3))*cos(theta__4) + (-sin(theta__1)*cos(theta__2)*sin(theta__3) - sin(theta__1)*sin(theta__2)*cos(theta__3))*sin(theta__4))*d__4 + (sin(theta__1)*cos(theta__2)*cos(theta__3) - sin(theta__1)*sin(theta__2)*sin(theta__3))*d__3 + sin(theta__1)*cos(theta__2)*d__2], [(sin(theta__2)*cos(theta__3) + cos(theta__2)*sin(theta__3))*cos(theta__4) + (-sin(theta__2)*sin(theta__3) + cos(theta__2)*cos(theta__3))*sin(theta__4), 0, -(sin(theta__2)*cos(theta__3) + cos(theta__2)*sin(theta__3))*sin(theta__4) + (-sin(theta__2)*sin(theta__3) + cos(theta__2)*cos(theta__3))*cos(theta__4), ((sin(theta__2)*cos(theta__3) + cos(theta__2)*sin(theta__3))*cos(theta__4) + (-sin(theta__2)*sin(theta__3) + cos(theta__2)*cos(theta__3))*sin(theta__4))*d__4 + (sin(theta__2)*cos(theta__3) + cos(theta__2)*sin(theta__3))*d__3 + sin(theta__2)*d__2 + d__1], [0, 0, 0, 1]])

"""
# s = "asdfasdf + asdfcos(90)asdfsadf - asdfasdfhjkl"
s = s.replace("Matrix(4, 4, [", "double T[4][4] = {\n\t")
s = s.replace("[", "{")
s = s.replace("]", "}")
s = s.replace("}, ", "},\n\t")
s = s.replace("__", "_")
s = s.replace("{4}{4}", "[4][4]")
s = s.replace("}})", "}};")
print(s)
