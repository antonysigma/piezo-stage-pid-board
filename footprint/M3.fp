# M3 Hole
#
#	1 millimeter = 39.3700787 mils
#	1 millimeter = 3937.00787 1/100th mils
#

# Pin[x y thickness clearance mask drillholedia name number flags]
# 
#	Item			Allowed Value		Explanation					Comment
# 	x				1/100th mils		pin x coord.	
# 	y				1/100th mils		pin y coord.	
# 	thickness		1/100th mils		copper diameter	
# 	clearance		1/100th mils		2*(cu to cu clearance)		if you want a 10 mil clearance, put 2000 (20 mils) here
# 	mask			1/100th mils		diameter of mask aperture	actual dia. of hole in mask
# 	drillholedia	1/100th mils		dia. of hole	
# 	name			string				arb. pin name	
# 	number			decimal integer		pin number used by nets/rats	
# 	flags			hex					xxx	

Element["" "M3 Hole" "" "" 11000 11000 0 0 0 100 ""]
(
	Attribute("gedasymbols::url" "http://www.gedasymbols.org/user/carlos_calzada_grau/footprints/holes/M3.fp")
	#Pin[0 0 22600 2000 23200 16000 "" "1" ""]
	Pin[0 0 5.74mm 2000 5.90mm 3.50mm "" "1" ""]

	Attribute("author" "Carlos Calzada Grau")
	Attribute("copyright" "2017 Carlos Calzada Grau")
	Attribute("use-license" "Unlimited")
	Attribute("dist-license" "GPL")
	Attribute("description" "M3 Hole")
)
