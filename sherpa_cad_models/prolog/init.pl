%%
%% Copyright (C) 2011 by Stefan Profanter
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_map_tools).
:- register_ros_package(iai_maps).
:- register_ros_package(sherpa_cad_models).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:# - owl_parser:owl_parse('/home/yazdani/work/ros/hydro/rosbuild_ws/workspace/sherpa_project/sherpa_cad_models/owl/sherpa_cad_models.owl', false, false, true).
:- owl_parse('package://sherpa_cad_models/owl/sherpa_cad_models.owl').
:- rdf_db:rdf_register_ns(cad, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

