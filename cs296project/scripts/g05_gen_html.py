import re

texfile = open("./doc/g05_report.tex", "r")
htmlfile = open('./doc/g05_report.html', 'w')
htmlfile.write('<head>\n' +'<title>Group5</title>\n' + '<style type="text/css">\n' +'</style>\n')
htmlfile.write('<link rel="stylesheet" type="text/css" href="g05_html.css">\n' + '</head>\n')

tex = texfile.read()
tex = tex.replace('\\\\','')
tex = tex.replace('\\nocite{*}','')
tex = tex.replace('\\bibliographystyle{plain}','')
tex = tex.replace('\\bibliography{bibtex}','')
tex = tex.replace('\\begin{tabular}{l l l}','')
tex = tex.replace('\\end{tabular}','')
tex = tex.replace('\\end{document}','')
tex = tex.replace('\\begin{center}','')
tex = tex.replace('\\end{center}','')
tex = tex.replace('\\begin{itemize}','')
tex = tex.replace('\\end{itemize}','')
tex = tex.replace('\\begin{enumerate}','')
tex = tex.replace('\\end{enumerate}','')
sections = tex.split("\\section")

def writep(lines) :
	item=0
	for line in lines :
		if line != line.replace('\\includegraphics','') :
			line=line.split('{')[1].replace('.eps}','')
			temp = line.strip()
			htmlfile.write('<img src="../' + temp + '.png">')
			line=''
		if line != line.replace('&','') :
			line = "\\item " + line.split('&')[2]
		if line != line.replace('\\item','') :
			if item == 0 :
				htmlfile.write('<ul>\n')
				item = 1
			htmlfile.write('<li>' + line.replace('\\item','') + '</li>\n')
		else :
			if item == 1 :
				htmlfile.write('</ul>\n')
				item = 0
			htmlfile.write('<p>' + line + '</p>\n')

for section in sections[1:] :
	subsections = section.split('\\subsection')
	temp=0
	secheader = subsections[0]

	if secheader != '' :
		lines = secheader.split('\n')
		head = lines[0]
		head = head.replace('{','')
		head = head.replace('}','')
		htmlfile.write('<h1>' + head + '</h1>\n')
		lines = lines[1:]
		writep(lines)

	subsections = subsections[1:]

	for subsection in subsections :
		lines = subsection.split('\n')
		subhead = lines[0]
		subhead = subhead.replace('{','')
		subhead = subhead.replace('}','')
		htmlfile.write('<h2>' + subhead + '</h2>\n')
		lines = lines[1:]
		writep(lines)

bibtexfile = open("./doc/bibtex.bib", "r")
bib = bibtexfile.read()
bib = bib.split('@misc')[1:]

htmlfile.write('<h1>References</h1>\n')
htmlfile.write('<ul>\n')

for entry in bib :
	entry = entry.split('"')
	htmlfile.write('<li><a href="')
	temp = entry[5].split('{')[1];
	temp = temp.split('}')[0];
	htmlfile.write(temp + '">' + entry[3] + '</a></li>\n')
	# print(entry)
# htmlfile.write('<li><a href="http://www.box2d.org/manual.html">Box2D Manual<\a><\li>\n')
# htmlfile.write('<li>http://www.box2d.org/documentation.html - Box2D Documentation<\li>\n')
# htmlfile.write('<li><\li>\n')
# htmlfile.write('<li><\li>\n')
# htmlfile.write('<li><\li>\n')
# htmlfile.write('<li><\li>\n')
# htmlfile.write('</ul>\n')
		
