#
#   description :   ミッションプランナーのファイルからコマンドと座標を取り出す
#   parameters  :
#                   in:FileName
#                   out:commands,lats, lons, alts

import csv

def mpReader( FileName ):

    commands=[]
    lats=[]
    lons=[]
    alts=[]

    #fileName = '/home/ardupilot/ardupilot/dev-app/day5/plane_NRT2Main.txt'
    #fileName = '/home/ardupilot/ardupilot/dev-app/day5/plane_Main2NRT.txt'

    with open( FileName, 'r' ) as inFile:
        lines = inFile.readlines()

    nRow = 0
    for s in lines:
        list = s.split("\t")
        if nRow > 1:        # MPファイルは2行スキップ
            commands.append( int( list[3] ) )
            lats.append( float( list[8] ) )
            lons.append( float( list[9] ) )
            alts.append( float( list[10] ) )
        nRow = nRow + 1

    return ( commands, lats, lons, alts )

# fileName = '/home/ardupilot/ardupilot/dev-app/day5/plane_NRT2Main.txt'
# ( commands, lats, lons, alts ) = mpReader( fileName )
# for command, lat, lon, alt in zip( commands, lats, lons, alts ):
#     print( command, lat, lon, alt )
