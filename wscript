## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    #print(bld.path.ant_glob('**/*.cc', src=True, bld=False, dir=False))
    env = bld.env_of_name('default')
    all_modules = [mod[len("ns3-"):] for mod in bld.env['NS3_MODULES']]

    obj = bld.create_ns3_program('vanet', all_modules)
    obj.source = bld.path.ant_glob('**/*.cc', src=True, bld=False, dir=False)
    #obj.path = obj.path.find_dir('vanet')
    #obj.find_sources_in_dirs('.')
    #for dirname in os.listdir("vanet"):
    #    if os.path.isdir(os.path.join("vanet", dirname)):
    #    obj.find_sources_in_dirs(dirname)
    #obj.find_sources_in_dirs('protocols')

