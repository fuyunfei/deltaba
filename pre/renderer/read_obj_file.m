function [V,F] = read_vertices_and_faces_from_obj_file(filename)
  % Reads a .obj mesh file and outputs the vertex and face list
  % assumes a 3D triangle mesh and ignores everything but:
  % v x y z and f i j k lines
  % Input:
  %  filename  string of obj file's path
  %
  % Output:
  %  V  number of vertices x 3 array of vertex positions
  %  F  number of faces x 3 array of face indices
  %
  V = zeros(0,3);
  F = zeros(0,3);
  vertex_index = 1;
  face_index = 1;   
  fid = fopen(filename,'rt');
  line = fgets(fid);
  while ischar(line)
    vertex = sscanf(line,'v %f %f %f');
    face = sscanf(line,'f %d %d %d %d');

    % see if line is vertex command if so add to vertices
    if(size(vertex)>0)
      V(vertex_index,:) = vertex;
      vertex_index = vertex_index+1;
   % see if line is simple face command if so add to faces
    elseif(size(face,1)==3)
      F(face_index,:) = face;
      face_index = face_index+1;
    % see if line is a face with normal indices command if so add to faces
    end

    line = fgets(fid);
  end
  fclose(fid);
end