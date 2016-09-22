function compile_renderer(LIB_PATH)
cmd='mex renderer.cpp depth.cpp Engine.cpp EngineOSG.cpp util/util.cpp -lGL -lX11 -losg -losgViewer -losgDB -losgGA -losgUtil -lOpenThreads -lGLU -Iutil/';
if exist('LIB_PATH','var') && ~isempty(LIB_PATH)
    libpath=fullfile(LIB_PATH,'lib');
    lib64path=fullfile(LIB_PATH,'lib64');
    incpath=fullfile(LIB_PATH,'include');
    GLPATH='Mesa-7.0.3';
    GLincpath=fullfile(LIB_PATH,fullfile(GLPATH,'include'));
    GLlibpath=fullfile(LIB_PATH,fullfile(GLPATH,'lib64'));
    cmd=[cmd sprintf(' -L%s -L%s -L%s -I%s -I%s',libpath,lib64path,GLlibpath,incpath,GLincpath)];
end
fprintf('Executing %s\n',cmd);
eval(cmd);
