# author: Amand Tihon
# email: amand.tihon@alrj.org
# dist-license: GPL3, http://www.gnu.org/licenses/gpl-3.0.txt
# use-license: unlimited


Element["" "" "C000" "" 20000 20000 12500 -15000 0 100 ""]
(
	Pin[-5000 0 8000 2000 8600 1500 "1" "1" ""]
	Pin[5000 0 8000 2000 8600 1500 "2" "2" ""]
	ElementLine [-5000 -10000 -5000 -5000 1500]
	ElementLine [-7500 -7500 -2500 -7500 1500]
	ElementArc [0 0 12500 12500 270 90 1500]
	ElementArc [0 0 12500 12500 180 90 1500]
	ElementArc [0 0 12500 12500 90 90 1500]
	ElementArc [0 0 12500 12500 0 90 1500]

	)
