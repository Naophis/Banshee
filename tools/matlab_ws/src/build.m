function build(tgt_model_name, result_dir)

    home = pwd;
    build_home = which(tgt_model_name);
    cd(fileparts(build_home));

    tgt_dir_name = append('gen_code_' , result_dir);

    mkdir('build');
    mkdir(tgt_dir_name);
    cd(tgt_dir_name);
    gen_code_dir = pwd;
    cd ..;
    tgtdir = strcat('../../', tgt_dir_name);

    cd build;

    slbuild(tgt_model_name);

    dir_list = dir;
    list_size = size(dir_list, 1);
    tgt_name = '';
    build_result_dir = append(tgt_model_name,'_ert');
    for i = 1:1:list_size
        dir_name = dir_list(i).name;
        res = contains(dir_name, build_result_dir);

        if res
            tgt_name = dir_name;
            break;
        end

    end

    cd(tgt_name);

    copyfile('*.c*', tgtdir);
    copyfile('*.h', tgtdir);

    cd(fileparts(build_home));
    cd ..;
    cd include;
    copyfile('*.h', gen_code_dir);

    cd(gen_code_dir);
    %    delete ert_main.c
    cp_from = append('../../../src/gen_code_',result_dir);
    copyfile('*.*', cp_from);
    cd(home);

end
