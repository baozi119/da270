from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add da217 src files.
src += Glob('sensor_mira_da217.c')
#src += Glob('libraries/da217.c')

# add da217 include path.
path  = [cwd]

# add src and include to group.
group = DefineGroup('da217', src, depend = ['PKG_USING_DA217'], CPPPATH = path)

Return('group')
