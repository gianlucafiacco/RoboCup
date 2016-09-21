GnuplotGUI::GnuplotGUI()
{
	if (getenv("DISPLAY") == NULL)
	{
		this->valid = false;
		throw GnuplotException("cannot find DISPLAY variable");
	}
	
	if (!this->get_program_path("gnuplot"))
	{
		this->valid = false;
		throw GnuplotException("Can't find gnuplot in your PATH");
	}
	
	this->gnucmd = popen("gnuplot","w");
	
	if (!this->gnucmd)
	{
		this->valid = false;
		throw GnuplotException("Could'nt open connection to gnuplot");
	}
	
	gs.x.min = 0.;
	gs.x.max = 0.;
	gs.y.min = 0.;
	gs.y.max = 0.;
	gs.xlabel = "X";
	gs.ylabel = "Y";
	tick_count = 0;
}

GnuplotGUI::~GnuplotGUI()
{
	if (pclose(this->gnucmd) == -1)
	{
		std::cerr << "Problem closing communication to gnuplot" << std::endl;
	}
	
	return;
}

#define PATH_MAXNAMESZ 4096

template<typename Container> void stringtok(Container &container, std::string const &in, const char * const delimiters = " \t\n")
{
	const std::string::size_type len = in.length();
	std::string::size_type i = 0;
	
	while (i < len)
	{
		// Eat leading whitespace.
		i = in.find_first_not_of(delimiters,i);
		
		if (i == std::string::npos)
		{
			// Nothing left but white space.
			return;
		}
		
		// Find the end of the token.
		std::string::size_type j = in.find_first_of(delimiters,i);
		
		// Push token.
		if (j == std::string::npos)
		{
			container.push_back(in.substr(i));
			
			return;
		}
		else
		{
			container.push_back(in.substr(i,j - i));
		}
		
		// Set up for next loop.
		i = j + 1;
	}
}

std::string& replaceAll(std::string& context, const std::string& from, const std::string& to)
{
	size_t lookHere = 0;
	size_t foundHere;
	
	while ((foundHere = context.find(from,lookHere)) != std::string::npos)
	{
		context.replace(foundHere,from.size(),to);
		lookHere = foundHere + to.size();
	}
	
	return context;
}

bool GnuplotGUI::get_program_path(const std::string pname)
{
	std::list<std::string> ls;
	char *path;
	
	path = getenv("PATH");
	
	if (!path)
	{
		std::cerr << "Path is not set" << std::endl;
		
		return false;
	}
	else
	{
		stringtok(ls,path,":");
		
		for (std::list<std::string>::const_iterator i = ls.begin(); i != ls.end(); i++)
		{
			std::string tmp = (*i) + "/" + pname;
			
			if (access(tmp.c_str(),X_OK ) == 0) return true;
		}
	}
	
	return false;
}

void GnuplotGUI::cmd(const std::string& cmdstr)
{
	__cmdstr = cmdstr;
	fputs(cmdstr.c_str(),this->gnucmd);
	fputs("\n",this->gnucmd);
	fflush(this->gnucmd);
	
	return;
}

void GnuplotGUI::set_ylabel(const std::string &label)
{
	gs.ylabel = label;
	
	return;
}

void GnuplotGUI::set_xlabel(const std::string &label)
{
	gs.xlabel = label;
	return;
}

// Set the xrange.
void GnuplotGUI::set_xrange(int from, int to)
{
	gs.x.min = from;
	gs.x.max = to;
}

// Set the yrange.
void GnuplotGUI::set_yrange(int from, int to)
{
	gs.y.min = from;
	gs.y.max = to;
}

template<typename P> void GnuplotGUI::put_label(int tag, const std::string& text, const P& location)
{
	Label& l = labels[tag];
	
	l.location = location.toString();
	replaceAll(l.location," ",",");
	l.lastValidTick = tick_count;
	
	if ( l.text.compare(text) != 0)
	{
		l.text = text;
	}
}

void GnuplotGUI::clear_labels()
{
	labels.clear();
	this->cmd("unset label\n");
}

template<typename P> void GnuplotGUI::put_object(int tag, const std::string& type, float width, float height, float rotation, const P& location)
{
	Object& o = objects[tag];
	o.location = location.toString();
	replaceAll(o.location," ",",");
	o.type = type;
	o.w = width;
	o.h = height;
	o.r = rotation;
	o.lastValidTick = tick_count;
}

void GnuplotGUI::clear_objects()
{
	objects.clear();
	this->cmd("unset object\n");
}

std::string GnuplotGUI::__get_labels()
{
	std::ostringstream oss;
	
	for (LabelMap::iterator it = labels.begin(); it != labels.end(); it++)
	{
		Label& lbl = it->second;
		size_t delta = tick_count - lbl.lastValidTick;
		
		if (delta == 0)
		{
			oss << "set label " << it->first << " at " << lbl.location << " '" << lbl.text	<< "';";
		}
		else if (delta <= GNUPLOT_TIME_WINDOW_FOR_VALID_DRAWING)
		{
			oss << "set label " << it->first << " at " << lbl.location << ";";
		}
		else
		{
			oss << "unset label " << it->first << ";";
			labels.erase(it);
		}
	}
	
	return oss.str();
}

std::string GnuplotGUI::__get_objects()
{
	std::ostringstream oss;
	
	for (ObjectMap::iterator it = objects.begin(); it != objects.end(); it++)
	{
		Object& obj = it->second;
		size_t delta = tick_count - obj.lastValidTick;
		
		if (delta == 0)
		{
			oss << "set object " << it->first << " " << obj.type << " at " << obj.location << " size " << obj.w << "," << obj.h << ";";
		}
		else if (delta <= GNUPLOT_TIME_WINDOW_FOR_VALID_DRAWING)
		{
			oss << "set object " << it->first << " " << obj.type << " at " << obj.location << ";";
		}
		else
		{
			oss << "unset object " << it->first << ";";
			objects.erase(it);
		}
	}
	
	return oss.str();
}

void GnuplotGUI::redraw()
{
	if (!canvas.size() > 0) return;
	
	std::ostringstream cmdstr;
	
	// Settings.
	if (gs.x.min != gs.x.max)
	{
		cmdstr << "set xrange [" << gs.x.min << ":" << gs.x.max << "];"; 
	}
	
	if (gs.y.min != gs.y.max)
	{
		cmdstr << "set yrange [" << gs.y.min << ":" << gs.y.max << "];";
	}
	
	cmdstr << "set xlabel '" << gs.xlabel << "';";
	cmdstr << "set ylabel '" << gs.ylabel << "';";
	cmdstr << __get_labels();
	cmdstr << __get_objects();
	
	cmdstr << std::endl;
	
	// Plot.
	cmdstr << "plot ";
	
	for (DrawableMap::const_iterator it = canvas.begin(); it != canvas.end(); it++)
	{
		it!=canvas.begin() && cmdstr << ",";
		cmdstr << " '-' ";
		
		if (it->second.title != "notitle")
		{
			cmdstr << " title '" << it->second.title << "'";
		}
		else
		{
			cmdstr << it->second.title;
		}
		
		cmdstr << " " << it->second.style;
	}
	
	cmdstr << std::endl;
	
	for (DrawableMap::const_iterator it = canvas.begin(); it != canvas.end(); it++)
	{
		cmdstr << it->second.obj << " # " << it->first << std::endl << "e" << std::endl;
	}
	
	cmdstr << std::endl;
	
	this->cmd(cmdstr.str());
	++tick_count;
}

void GnuplotGUI::draw_point(const Point& p, const std::string& tag, const std::string& title, const std::string& style)
{
	canvas.insert(make_pair(tag,Drawable(p,title,style)));
}

template<typename P> void GnuplotGUI::draw_points(const std::vector<P>& vp, const std::string& tag, const std::string& title, const std::string& style)
{
	std::ostringstream obj;
	
	for (typename std::vector<P>::const_iterator it = vp.begin(); it != vp.end(); it++)
	{
		obj << it->toString() << std::endl;
	}
	
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		std::ostringstream sstyle;
		sstyle << "w p " << style;
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
}

void GnuplotGUI::__draw_line(const CanvasObject& o, const std::string& tag, const std::string& title, const std::string& style)
{
	std::ostringstream sstyle;
	sstyle << "w l " << style;
	
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(o.toString());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(o,title,sstyle.str())));
	}
}

template<typename P> void GnuplotGUI::draw_line(const Line<P>& l, const std::string& tag, const std::string& title, const std::string& style)
{
	__draw_line(l, tag, title, style);
}

template<typename P> void GnuplotGUI::draw_line(const P& p0, const P& p1, const std::string& tag, const std::string& title, const std::string& style)
{
	Line<P> l(p0,p1);
	__draw_line(l, tag, title, style);
}

template<typename P> void GnuplotGUI::draw_lines(const std::vector<Line<P> >& vl, const std::string& tag, const std::string& title, const std::string& style)
{
	std::ostringstream sstyle, obj;
	sstyle << "w l " << style;
	
	for (typename std::vector<Line<P> >::const_iterator it = vl.begin(); it != vl.end(); it++)
	{
		obj << it->toString() << std::endl << std::endl;
	}
	
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
}

template<typename P> void GnuplotGUI::draw_path(const Path<P>& p, const std::string& tag, const std::string& title, const std::string& style)
{
	__draw_line(p, tag, title, style);
}

template<typename P> void GnuplotGUI::draw_path(const std::vector<P>& vp, const std::string& tag, const std::string& title, const std::string& style)
{
	std::ostringstream sstyle, obj;
	sstyle << "w l " << style;
	
	for (typename std::vector<P>::const_iterator it = vp.begin(); it != vp.end(); it++)
	{
		obj << it->toString() << std::endl;
	}
	
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
}

template<typename P> void GnuplotGUI::draw_paths(const std::vector<Path<P> >& vp, const std::string& tag, const std::string& title, const std::string& style)
{
	std::ostringstream sstyle, obj;
	sstyle << "w l " << style;
	
	for (typename std::vector<Path<P> >::const_iterator it = vp.begin(); it != vp.end(); it++)
	{
		obj << it->toString() << std::endl;
	}
	
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
}
