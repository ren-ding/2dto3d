function OBJ = read_obj( fullfilename )

if(exist('fullfilename','var')==0)
    [filename, filefolder] = uigetfile('*.obj', 'Read obj-file');
    fullfilename = [filefolder filename];
end
filefolder = fileparts( fullfilename);

disp(['Reading Object file : ' fullfilename]); 

fid = fopen(fullfilename,'r');
if (fid<0)
  error(['can not open file: ' fullfilename]);
  return ;
end

% Pre-Parse pass: Load the whole file into a matlab matrix and then count
% number of vertices et al. to quickly determine the storage requirements.
preobj = fread(fid, inf, 'uint8=>char')';
pre_vnum = length(findstr(preobj, 'v '));
pre_vtnum = length(findstr(preobj, 'vt '));
pre_vnnum = length(findstr(preobj, 'vn '));
pre_f3num = length(findstr(preobj, 'f '));

vertices=zeros(pre_vnum,3); 
vertices_texture=zeros(pre_vtnum,2); 
vertices_normal=zeros(pre_vnnum,3);

%triangle
vertices_facelist=zeros(pre_f3num,3); 
vertices_texturelist=zeros(pre_f3num,3); 
vertices_normallist=zeros(pre_f3num,3);

%quad
% vertices_facelist=zeros(pre_f3num,4); 
% vertices_texturelist=zeros(pre_f3num,4); 
% vertices_normallist=zeros(pre_f3num,4);

nv=0;
nvt=0;
nvn=0;
nf=0;

% Rewind to beginning of file in preparation of real data parse pass:
frewind(fid);

while ~feof(fid)
  Ln=fgets(fid);
  Ln=removespace(Ln);
  objtype=sscanf(Ln,'%s',1);
  l=length(Ln);
  if l==0  % isempty(s) ; 
    continue;
  end
  
  switch objtype
    case '#' % comment
      %disp(Ln);
    case 'v' % vertex
      v=sscanf(Ln(2:end),'%f');
      nv = nv+1;
      vertices(nv,:) = v;
    case 'vt'	% textures
      vt=sscanf(Ln(3:end),'%f');
      nvt = nvt+1;
      vertices_texture(nvt,:)=vt;
    case 'vn'	% textures
      vn=sscanf(Ln(3:end),'%f');
      nvn = nvn+1;
      vertices_normal(nvn,:)=vn;
    case 'f' % faces
      %nvrts=length(findstr(Ln,' ')); % spaces as a predictor of n vertices
      %slashpat=findstr(Ln,'/');      % slash position
      Ln=Ln(3:end);                  % get rid of the f
      nf = nf+1;
      quad_data=sscanf(Ln,'%f/%f/%f');
      
      vertices_facelist(nf,:) = quad_data([1 4 7]);
      vertices_texturelist(nf,:) = quad_data([2 5 8]);
      vertices_normallist(nf,:) = quad_data([3 6 9]);
    case 'g' % sub mesh
        %disp(Ln);
    case 'mtllib' % material library
        %disp(Ln);
    case 'usemtl' % use this material name
        %disp(Ln);
    case 'l' % Line
        %disp(Ln);
    case 's' %smooth shading across polygons
        %disp(Ln);
  end
  
end

fclose(fid);

OBJ.vertices=vertices(1:nv,:);
OBJ.vertices_texture=vertices_texture(1:nvt,:);
OBJ.vertices_normal=vertices_normal(1:nvn,:);
OBJ.vertices_facelist=vertices_facelist(1:nf,:);
OBJ.vertices_texturelist=vertices_texturelist(1:nf,:);
OBJ.vertices_normallist=vertices_normallist(1:nf,:);

end

function Lyn=removespace(Lyn)
% A not an elegant way to remove
% surplus space
Lyn=strtrim(Lyn);
Lyn=strrep(Lyn,'       ',' '); % 8-2 .. 12-6  
Lyn=strrep(Lyn,'    ',' '); % 5-2 6-3 4-1
Lyn=strrep(Lyn,'  ',' '); % 3-2 2-1
Lyn=strrep(Lyn,'  ',' '); 
Lyn=strrep(Lyn,char([13 10]),''); % remove cr/lf 
Lyn=strrep(Lyn,char([10]),''); % remove lf
end
