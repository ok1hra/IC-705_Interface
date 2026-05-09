'use strict';
/**
 * DXCC prefix lookup engine
 *
 * dxcc (c) 2007 Fabian Kurz, DJ1YFK — http://fkurz.net/ham/dxcc/
 * Based on cty.dat country file by Jim Reisert, AD1C — http://country-files.com/
 * Free software under GNU General Public License.
 *
 * JavaScript port for IC-705 Interface contest logger.
 */

(function (global) {

  // ── CTY data ────────────────────────────────────────────────────────────────
  // [name, cqZone, ituZone, continent, lat(+N/-S), lon(+E/-W),
  //  utcOffset(cty.dat conv: +W/-E), mainPrefix, [aliases]]
  // aliases: '=' prefix = exact callsign match

  const CTY_RAW = [
    // ── Europe ────────────────────────────────────────────────────────────────
    ['Sov. Mil. Order of Malta',15,28,'EU',41.9,12.4,-1,'1A',['1A']],
    ['Monaco',14,27,'EU',43.7,7.4,-1,'3A',['3A']],
    ['Montenegro',15,28,'EU',42.8,19.5,-1,'4O',['4O']],
    ['Croatia',15,28,'EU',45.2,15.5,-1,'9A',['9A']],
    ['Malta',15,28,'EU',35.9,14.4,-1,'9H',['9H']],
    ['Andorra',14,27,'EU',42.5,1.5,-1,'C3',['C3']],
    ['Portugal',14,37,'EU',39.5,-8.0,0,'CT',['CT','CR','CS','CQ']],
    ['Madeira Is',33,36,'AF',32.6,-16.9,0,'CT3',['CT3','CS3','CR3','CQ3']],
    ['Azores',14,36,'EU',37.8,-25.6,1,'CU',['CU']],
    ['Germany',14,28,'EU',51.2,10.0,-1,'DL',['DA','DB','DC','DD','DE','DF','DG','DH','DJ','DK','DL','DM','DN','DO','DR']],
    ['Bosnia-Herzegovina',15,28,'EU',44.0,17.5,-1,'E7',['E7']],
    ['Spain',14,37,'EU',40.4,-3.7,-1,'EA',['EA','EB','EC','ED','EE','EF','EG','EH','AM']],
    ['Balearic Is',14,37,'EU',39.6,3.0,-1,'EA6',['EA6','EB6','EC6','EF6','EG6','EH6']],
    ['Canary Is',33,36,'AF',28.1,-15.4,0,'EA8',['EA8','EB8','EC8','EF8','EG8','EH8']],
    ['Ceuta & Melilla',33,37,'AF',35.9,-5.3,-1,'EA9',['EA9','EB9','EC9','EF9','EG9','EH9']],
    ['Ireland',14,27,'EU',53.1,-8.0,0,'EI',['EI','EJ']],
    ['Moldova',16,29,'EU',47.0,28.5,-2,'ER',['ER']],
    ['Estonia',15,29,'EU',58.7,25.5,-2,'ES',['ES']],
    ['Belarus',16,29,'EU',53.5,27.5,-2,'EW',['EU','EV','EW']],
    ['Ukraine',16,29,'EU',50.0,31.5,-2,'UT',['EM','EO','UR','US','UT']],
    ['France',14,27,'EU',46.0,2.0,-1,'F',['F','TM']],
    ['England',14,27,'EU',52.0,-1.5,0,'G',['G','M','2E','2I','2U','2W']],
    ['Isle of Man',14,27,'EU',54.2,-4.5,0,'GD',['GD','MD']],
    ['Northern Ireland',14,27,'EU',54.7,-6.7,0,'GI',['GI','MI']],
    ['Jersey',14,27,'EU',49.2,-2.1,0,'GJ',['GJ','MJ']],
    ['Scotland',14,27,'EU',57.0,-4.0,0,'GM',['GM','MM']],
    ['Guernsey',14,27,'EU',49.5,-2.6,0,'GU',['GU','MU']],
    ['Wales',14,27,'EU',52.3,-3.5,0,'GW',['GW','MW']],
    ['Hungary',15,28,'EU',47.1,19.0,-1,'HA',['HA','HG']],
    ['Switzerland',14,28,'EU',47.0,8.3,-1,'HB',['HB']],
    ['Liechtenstein',14,28,'EU',47.1,9.5,-1,'HB0',['HB0']],
    ['Vatican City',15,28,'EU',41.9,12.5,-1,'HV',['HV']],
    ['Italy',15,28,'EU',42.8,12.8,-1,'I',['I','IK','IQ','IR','IU','IW','IY','IZ']],
    ['Sardinia',15,28,'EU',39.9,9.1,-1,'IS',['IS','IM0']],
    ['African Italy',33,37,'AF',35.9,12.4,-1,'IG9',['IG9','IH9']],
    ['Svalbard',18,18,'EU',78.0,16.0,-1,'JW',['JW']],
    ['Jan Mayen',18,18,'EU',71.1,-8.3,1,'JX',['JX']],
    ['Norway',14,18,'EU',61.0,9.5,-1,'LA',['LA','LB','LC','LD','LE','LF','LG','LH','LI','LJ','LK','LM','LN']],
    ['Luxembourg',14,27,'EU',49.7,6.2,-1,'LX',['LX']],
    ['Lithuania',15,29,'EU',55.2,24.0,-2,'LY',['LY']],
    ['Bulgaria',20,28,'EU',42.7,25.5,-2,'LZ',['LZ']],
    ['Austria',15,28,'EU',47.5,14.5,-1,'OE',['OE']],
    ['Finland',18,18,'EU',64.0,26.0,-2,'OH',['OF','OG','OH','OI']],
    ['Aland Is',18,18,'EU',60.2,20.0,-2,'OH0',['OH0','OG0','OF0']],
    ['Market Reef',18,18,'EU',60.3,19.1,-2,'OJ0',['OJ0']],
    ['Czech Republic',15,28,'EU',50.0,16.0,-1,'OK',['OK','OL']],
    ['Slovak Republic',15,28,'EU',48.7,19.5,-1,'OM',['OM']],
    ['Belgium',14,27,'EU',50.7,4.5,-1,'ON',['ON','OO','OP','OQ','OR','OS','OT']],
    ['Faroe Is',14,18,'EU',62.1,-7.0,0,'OY',['OY']],
    ['Denmark',14,18,'EU',56.0,10.0,-1,'OZ',['5P','5Q','OU','OV','OW','OZ']],
    ['Netherlands',14,27,'EU',52.3,5.3,-1,'PA',['PA','PB','PC','PD','PE','PF','PG','PH','PI']],
    ['Slovenia',15,28,'EU',46.0,15.0,-1,'S5',['S5']],
    ['Sweden',14,18,'EU',60.0,16.0,-1,'SM',['SA','SB','SC','SD','SE','SF','SG','SH','SI','SJ','SK','SL','SM']],
    ['Poland',15,28,'EU',52.1,19.5,-1,'SP',['3Z','HF','SN','SO','SP','SQ','SR']],
    ['Greece',20,28,'EU',38.3,22.5,-2,'SV',['J4','SV','SW','SX','SY','SZ']],
    ['Dodecanese',20,28,'EU',36.2,28.0,-2,'SV5',['SV5','SW5','SX5','J45']],
    ['Crete',20,28,'EU',35.2,24.9,-2,'SV9',['SV9','SW9','SX9','J49']],
    ['San Marino',15,28,'EU',43.9,12.5,-1,'T7',['T7']],
    ['Turkey',20,29,'AS',39.0,35.0,-3,'TA',['TA','TB','TC','YM']],
    ['Iceland',40,17,'EU',64.9,-18.9,0,'TF',['TF']],
    ['Corsica',15,28,'EU',42.0,9.0,-1,'TK',['TK']],
    ['European Russia',16,29,'EU',55.0,40.0,-3,'UA',['R','RA','RB','RC','RD','RE','RF','RG','RH','RI','RJ','RK','RL','RM','RN','RO','RP','RQ','RS','RT','RU','RV','RW','RX','RY','RZ','UA','UB','UC','UD','UE','UF','UG','UH','UI']],
    ['Kaliningrad',15,29,'EU',54.7,20.5,-2,'UA2',['UA2','RA2','RK2']],
    ['Asiatic Russia',17,30,'AS',55.0,80.0,-6,'UA9',['UA9','UA0','RA9','RA0','R9','R0']],
    ['Latvia',15,29,'EU',57.0,25.0,-2,'YL',['YL']],
    ['Romania',20,28,'EU',46.0,25.0,-2,'YO',['YO','YP','YQ','YR']],
    ['Serbia',15,28,'EU',44.0,21.0,-1,'YU',['4N','YT','YU']],
    ['North Macedonia',15,28,'EU',41.6,21.8,-1,'Z3',['Z3']],
    ['Albania',15,28,'EU',41.3,20.0,-1,'ZA',['ZA']],
    ['Gibraltar',14,37,'EU',36.1,-5.4,0,'ZB',['ZB','ZG']],
    ['Kosovo',15,28,'EU',42.7,21.2,-1,'Z6',['Z6']],
    // ── North America ─────────────────────────────────────────────────────────
    ['Alaska',1,1,'NA',64.0,-153.0,9,'KL',['AL','KL','NL','WL']],
    ['Hawaii',31,61,'OC',21.4,-158.1,10,'KH6',['AH6','KH6','NH6','WH6']],
    ['Mariana Is',27,64,'OC',15.2,145.7,-10,'KH0',['AH0','KH0','NH0','WH0']],
    ['Johnston Is',31,61,'OC',16.7,-169.5,10,'KH3',['AH3','KH3','NH3','WH3']],
    ['Midway Is',31,61,'OC',28.2,-177.4,11,'KH4',['AH4','KH4','NH4','WH4']],
    ['Palmyra & Jarvis Is',31,61,'OC',5.9,-162.1,11,'KH5',['AH5','KH5','NH5','WH5']],
    ['Baker & Howland Is',31,61,'OC',0.2,-176.5,12,'KH1',['AH1','KH1','NH1','WH1']],
    ['Wake Is',31,65,'OC',19.3,166.6,-12,'KH9',['AH9','KH9','NH9','WH9']],
    ['Guam',27,64,'OC',13.5,144.8,-10,'KH2',['AH2','KH2','NH2','WH2']],
    ['American Samoa',32,62,'OC',-14.3,-170.8,11,'KH8',['AH8','KH8','NH8','WH8']],
    ['Puerto Rico',8,11,'NA',18.3,-66.5,4,'KP4',['AH8S','KP4','NP4','WP4']],
    ['US Virgin Is',8,11,'NA',18.3,-64.9,4,'KP2',['KP2','NP2','WP2']],
    ['Navassa Is',8,11,'NA',18.4,-75.0,5,'KP1',['KP1','NP1','WP1']],
    ['Desecheo Is',8,11,'NA',18.4,-67.5,5,'KP5',['KP5','NP5','WP5']],
    ['USA',5,6,'NA',37.5,-98.0,5,'K',['AA','AB','AC','AD','AE','AF','AG','AH','AI','AJ','AK','K','N','W']],
    ['Canada',5,4,'NA',45.0,-80.0,5,'VE',['CF','CG','VA','VB','VC','VE','VG','VX','VY','XJ','XK','XL','XM','XN','XO']],
    ['Greenland',40,5,'NA',72.0,-40.0,3,'OX',['OX','XP']],
    ['St Pierre & Miquelon',5,9,'NA',46.8,-56.2,3,'FP',['FP']],
    ['Mexico',6,10,'NA',23.0,-102.0,6,'XE',['4A','XA','XB','XC','XD','XE','XF','4B','4C','4D']],
    ['Revillagigedo',7,10,'NA',18.7,-110.9,7,'XF4',['XF4']],
    ['Clipperton Is',7,10,'NA',10.3,-109.2,8,'FO/C',['TX0C','FO/C']],
    ['Guatemala',7,11,'NA',15.5,-90.3,6,'TG',['TG','TD']],
    ['Belize',7,11,'NA',17.2,-88.7,6,'V3',['V3']],
    ['Honduras',7,11,'NA',15.0,-86.5,6,'HR',['HQ','HR']],
    ['El Salvador',7,11,'NA',13.7,-89.1,6,'YS',['HU','YS']],
    ['Nicaragua',7,11,'NA',13.0,-85.0,6,'YN',['H6','H7','YN']],
    ['Costa Rica',7,11,'NA',10.0,-84.0,6,'TI',['TE','TI']],
    ['Panama',7,11,'NA',9.0,-80.0,5,'HP',['3E','3F','H3','H8','H9','HO','HP']],
    ['Cuba',8,11,'NA',22.0,-80.0,5,'CO',['CL','CM','CO','CU','T4']],
    ['Cayman Is',8,11,'NA',19.3,-81.4,5,'ZF',['ZF']],
    ['Jamaica',8,11,'NA',17.9,-76.9,5,'6Y',['6Y']],
    ['Haiti',8,11,'NA',19.1,-72.3,5,'HH',['4V','HH']],
    ['Dominican Republic',8,11,'NA',19.0,-70.5,4,'HI',['HI']],
    ['Turks & Caicos Is',8,11,'NA',21.8,-71.7,5,'VP5',['VP5']],
    ['Bahamas',8,11,'NA',24.3,-76.0,5,'C6',['C6']],
    ['Bermuda',5,11,'NA',32.3,-64.8,4,'VP9',['VP9']],
    ['St Kitts & Nevis',8,11,'NA',17.3,-62.7,4,'V4',['V4']],
    ['Antigua & Barbuda',8,11,'NA',17.1,-61.8,4,'V2',['V2']],
    ['Montserrat',8,11,'NA',16.7,-62.2,4,'VP2M',['VP2M']],
    ['Guadeloupe',8,11,'NA',16.3,-61.6,4,'FG',['FG']],
    ['Martinique',8,11,'NA',14.6,-61.0,4,'FM',['FM']],
    ['Saint Martin',8,11,'NA',18.1,-63.1,4,'FS',['FS']],
    ['St Barthelemy',8,11,'NA',17.9,-62.8,4,'FJ',['FJ']],
    ['Sint Maarten',8,11,'NA',18.0,-63.1,4,'PJ7',['PJ7']],
    ['Saba & St Eustatius',8,11,'NA',17.6,-63.2,4,'PJ5',['PJ5','PJ6']],
    ['Bonaire',9,11,'SA',12.2,-68.3,4,'PJ4',['PJ4']],
    ['Curacao',9,11,'SA',12.2,-69.0,4,'PJ2',['PJ2']],
    ['Aruba',9,11,'SA',12.5,-70.0,4,'P4',['P4']],
    ['Dominica',8,11,'NA',15.4,-61.4,4,'J7',['J7']],
    ['St Lucia',8,11,'NA',14.0,-61.0,4,'J6',['J6']],
    ['St Vincent',8,11,'NA',13.2,-61.2,4,'J8',['J8']],
    ['Grenada',8,11,'NA',12.1,-61.7,4,'J3',['J3']],
    ['Barbados',8,11,'NA',13.2,-59.5,4,'8P',['8P']],
    ['Trinidad & Tobago',9,11,'SA',10.5,-61.4,4,'9Y',['9Y','9Z']],
    ['Anguilla',8,11,'NA',18.2,-63.1,4,'VP2E',['VP2E']],
    ['British Virgin Is',8,11,'NA',18.4,-64.6,4,'VP2V',['VP2V']],
    // ── South America ─────────────────────────────────────────────────────────
    ['Brazil',11,15,'SA',-10.0,-53.0,3,'PY',['PP','PQ','PR','PS','PT','PU','PV','PW','PX','PY','ZV','ZW','ZX','ZY','ZZ']],
    ['Argentina',13,14,'SA',-34.0,-64.0,3,'LU',['AY','LO','LP','LQ','LR','LS','LT','LU','LV','LW','AZ']],
    ['Chile',12,14,'SA',-30.0,-71.0,4,'CE',['3G','CA','CB','CC','CD','CE','XQ','XR']],
    ['Uruguay',13,14,'SA',-33.0,-56.0,3,'CX',['CV','CW','CX']],
    ['Paraguay',11,14,'SA',-23.0,-58.0,4,'ZP',['ZP']],
    ['Bolivia',10,14,'SA',-17.0,-65.0,4,'CP',['CP']],
    ['Peru',10,12,'SA',-10.0,-76.0,5,'OA',['4T','OA','OB','OC']],
    ['Ecuador',10,12,'SA',-2.0,-78.0,5,'HC',['HC','HD']],
    ['Colombia',9,12,'SA',4.6,-74.0,5,'HK',['5J','5K','HJ','HK']],
    ['Venezuela',9,12,'SA',8.0,-66.0,4,'YV',['4M','YV','YW','YX','YY']],
    ['Guyana',9,12,'SA',5.0,-59.0,4,'8R',['8R']],
    ['Suriname',9,12,'SA',4.0,-56.0,3,'PZ',['PZ']],
    ['French Guiana',9,12,'SA',4.9,-52.3,3,'FY',['FY']],
    ['Falkland Is',13,16,'SA',-51.7,-59.5,3,'VP8',['VP8']],
    ['South Georgia I',13,73,'SA',-54.3,-36.5,2,'VP8/G',['=VP8/G']],
    ['South Sandwich Is',13,73,'SA',-57.0,-27.0,2,'VP8/S',['=VP8/S']],
    ['South Orkney Is',13,73,'AN',-60.6,-45.6,3,'VP8/O',['=VP8/O']],
    ['Peter I Is',12,72,'AN',-68.8,-90.6,4,'3Y/P',['=3Y/P']],
    ['Bouvet I',38,67,'AF',-54.4,3.4,-1,'3Y/B',['=3Y/B']],
    // ── Africa ────────────────────────────────────────────────────────────────
    ['Morocco',33,37,'AF',32.0,-5.0,0,'CN',['5C','5D','5E','5F','5G','CN']],
    ['Algeria',33,37,'AF',28.0,2.5,-1,'7X',['7T','7U','7V','7W','7X','7Y']],
    ['Tunisia',33,37,'AF',34.0,9.0,-1,'3V',['3V','TS']],
    ['Libya',33,38,'AF',27.0,17.0,-1,'5A',['5A']],
    ['Egypt',34,38,'AF',27.0,29.5,-2,'SU',['6A','6B','SS','SU']],
    ['Sudan',34,48,'AF',15.0,30.0,-3,'ST',['6T','6U','ST']],
    ['South Sudan',34,48,'AF',7.0,30.0,-3,'Z8',['Z8']],
    ['Ethiopia',34,48,'AF',9.0,38.8,-3,'ET',['ET','9E','9F']],
    ['Eritrea',34,48,'AF',15.3,39.0,-3,'E3',['E3']],
    ['Djibouti',34,48,'AF',11.8,43.0,-3,'J2',['J2']],
    ['Somalia',34,48,'AF',7.0,46.0,-3,'T5',['6O','T5']],
    ['Kenya',37,48,'AF',1.1,37.9,-3,'5Z',['5Z']],
    ['Uganda',36,48,'AF',1.0,32.2,-3,'5X',['5X']],
    ['Tanzania',37,53,'AF',-6.8,36.7,-3,'5H',['5H','5I']],
    ['Rwanda',36,52,'AF',-2.0,30.0,-2,'9X',['9X']],
    ['Burundi',36,52,'AF',-3.4,29.9,-2,'9U',['9U']],
    ['Congo (DRC)',36,52,'AF',-2.9,23.6,-1,'9Q',['9O','9P','9Q','9R','9S','9T']],
    ['Congo Republic',36,52,'AF',-1.0,15.5,-1,'TN',['TN']],
    ['Gabon',36,52,'AF',-0.7,11.8,-1,'TR',['TR']],
    ['Cameroon',36,47,'AF',5.5,12.3,-1,'TJ',['TJ']],
    ['Central African Republic',36,47,'AF',6.0,20.0,-1,'TL',['TL']],
    ['Chad',35,47,'AF',12.0,17.0,-1,'TT',['TT']],
    ['Niger',35,46,'AF',17.3,8.0,-1,'5U',['5U']],
    ['Nigeria',35,46,'AF',9.0,8.0,-1,'5N',['5N','5O']],
    ['Benin',35,46,'AF',9.4,2.4,-1,'TY',['TY']],
    ['Togo',35,46,'AF',8.6,1.2,0,'5V',['5V']],
    ['Ghana',35,46,'AF',7.9,-1.2,0,'9G',['9G']],
    ['Cote d\'Ivoire',35,46,'AF',6.8,-5.3,0,'TU',['TU']],
    ['Liberia',35,46,'AF',6.3,-10.8,0,'EL',['A8','EL']],
    ['Sierra Leone',35,46,'AF',8.5,-11.8,0,'9L',['9L']],
    ['Guinea',35,46,'AF',11.5,-11.5,0,'3X',['3X']],
    ['Guinea-Bissau',35,46,'AF',12.0,-15.0,0,'J5',['J5']],
    ['Senegal',35,46,'AF',14.4,-14.7,0,'6W',['6V','6W']],
    ['Gambia',35,46,'AF',13.4,-15.4,0,'C5',['C5']],
    ['Cape Verde',35,46,'AF',15.1,-23.6,1,'D4',['D4']],
    ['Mauritania',35,46,'AF',20.3,-12.1,0,'5T',['5T']],
    ['Mali',35,46,'AF',17.5,-3.0,0,'TZ',['TZ']],
    ['Burkina Faso',35,46,'AF',13.0,-2.0,0,'XT',['XT']],
    ['Western Sahara',33,46,'AF',24.0,-13.0,0,'S0',['S0']],
    ['Sao Tome & Principe',36,47,'AF',0.2,6.6,-1,'S9',['S9']],
    ['Equatorial Guinea',36,47,'AF',1.5,10.3,-1,'3C',['3C']],
    ['Annobon I',36,52,'AF',-1.4,5.6,-1,'3C0',['3C0']],
    ['South Africa',38,57,'AF',-29.0,25.0,-2,'ZS',['S8','ZR','ZS','ZT','ZU']],
    ['Prince Edward & Marion Is',38,57,'AF',-46.9,37.6,-3,'ZS8',['ZS8']],
    ['Namibia',38,57,'AF',-22.0,17.0,-1,'V5',['V5']],
    ['Botswana',38,57,'AF',-22.2,24.0,-2,'A2',['A2','8O']],
    ['Zimbabwe',38,53,'AF',-19.0,30.0,-2,'Z2',['Z2']],
    ['Zambia',37,53,'AF',-15.0,28.0,-2,'9J',['9I','9J']],
    ['Mozambique',37,53,'AF',-18.5,35.5,-2,'C9',['C8','C9']],
    ['Malawi',37,53,'AF',-14.0,33.8,-2,'7Q',['7Q']],
    ['Eswatini',38,57,'AF',-26.5,31.5,-2,'3DA',['3DA','3DB','3DC','3DD','3DE','3DF']],
    ['Lesotho',38,57,'AF',-29.5,28.3,-2,'7P',['7P']],
    ['Madagascar',39,53,'AF',-19.9,46.9,-3,'5R',['5R','6X']],
    ['Comoros',39,53,'AF',-11.6,43.3,-3,'D6',['D6']],
    ['Mayotte',39,53,'AF',-12.8,45.2,-3,'FH',['FH']],
    ['Reunion',39,53,'AF',-21.1,55.5,-4,'FR',['FR']],
    ['Mauritius',39,53,'AF',-20.2,57.5,-4,'3B8',['3B8']],
    ['Rodriguez I',39,53,'AF',-19.7,63.4,-4,'3B9',['3B9']],
    ['Agalega & St Brandon',39,53,'AF',-10.4,56.6,-4,'3B6',['3B6']],
    ['Crozet I',38,68,'AF',-46.4,51.9,-4,'FT/W',['FT5W','FT/W']],
    ['Kerguelen Is',29,68,'AF',-49.4,69.5,-5,'FT/X',['FT5X','FT/X']],
    ['Amsterdam & St Paul Is',29,68,'AF',-38.7,77.5,-5,'FT/Z',['FT5Z','FT/Z']],
    ['Heard I',29,68,'OC',-53.1,73.5,-5,'VK0H',['VK0H']],
    // ── Middle East & Asia ────────────────────────────────────────────────────
    ['Israel',20,39,'AS',31.5,34.9,-2,'4X',['4X','4Z']],
    ['Palestinian Auth',20,39,'AS',31.9,35.4,-2,'E4',['E4']],
    ['Lebanon',20,39,'AS',33.9,35.5,-2,'OD',['OD']],
    ['Syria',20,39,'AS',34.8,38.9,-2,'YK',['YK']],
    ['Iraq',21,39,'AS',33.0,44.0,-3,'YI',['HN','YI']],
    ['Jordan',20,39,'AS',31.9,36.0,-2,'JY',['JY']],
    ['Saudi Arabia',21,39,'AS',24.0,45.0,-3,'HZ',['7Z','HZ']],
    ['Kuwait',21,39,'AS',29.2,47.6,-3,'9K',['9K']],
    ['Bahrain',21,39,'AS',26.0,50.6,-3,'A9',['A9']],
    ['Qatar',21,39,'AS',25.3,51.2,-3,'A7',['A7']],
    ['UAE',21,39,'AS',24.0,54.0,-4,'A6',['A6']],
    ['Oman',21,39,'AS',23.6,58.0,-4,'A4',['A4']],
    ['Yemen',21,39,'AS',15.6,44.1,-3,'7O',['7O']],
    ['Iran',21,40,'AS',32.5,54.0,-3.5,'EP',['EP','EQ']],
    ['Pakistan',21,41,'AS',30.0,70.0,-5,'AP',['6P','AP']],
    ['Afghanistan',21,28,'AS',33.0,65.0,-4.5,'YA',['T6','YA']],
    ['Bhutan',26,41,'AS',27.5,90.5,-6,'A5',['A5']],
    ['Nepal',26,42,'AS',28.0,84.0,-5.75,'9N',['9N']],
    ['India',26,41,'AS',20.0,77.0,-5.5,'VU',['8T','8U','AT','AU','AV','AW','VU','VX']],
    ['Andaman & Nicobar Is',26,49,'AS',12.5,92.8,-5.5,'VU4',['VU4']],
    ['Lakshadweep Is',26,41,'AS',11.2,72.6,-5.5,'VU7',['VU7']],
    ['Sri Lanka',26,41,'AS',7.6,80.7,-5.5,'4S',['4S','DJ8','YL2']],
    ['Maldives',26,41,'AS',4.2,73.5,-5,'8Q',['8Q']],
    ['Bangladesh',26,42,'AS',24.0,90.0,-6,'S2',['S2','S3']],
    ['Myanmar',26,49,'AS',21.0,96.0,-6.5,'XZ',['XY','XZ']],
    ['Thailand',26,49,'AS',15.0,101.0,-7,'HS',['E2','HS']],
    ['Laos',26,49,'AS',18.0,103.0,-7,'XW',['XW']],
    ['Vietnam',26,49,'AS',16.1,107.8,-7,'3W',['3W','XV']],
    ['Cambodia',26,49,'AS',12.0,105.0,-7,'XU',['XU']],
    ['Malaysia (W)',28,54,'AS',3.7,109.7,-8,'9M2',['9M2','9W2']],
    ['Malaysia (E)',28,54,'OC',5.9,116.1,-8,'9M6',['9M6','9M8','9W6']],
    ['Singapore',28,54,'AS',1.3,103.8,-8,'9V',['9V']],
    ['Indonesia',28,54,'OC',-5.0,120.0,-7,'YB',['7A','7B','7C','7D','7E','7F','7G','7H','7I','PK','PL','PM','PN','PO','YB','YC','YD','YE','YF','YG','YH']],
    ['East Timor',28,54,'OC',-8.8,125.6,-9,'4W',['4W']],
    ['Philippines',27,50,'OC',12.7,122.9,-8,'DU',['4D','4E','4F','4G','4H','4I','DU','DV','DW','DX','DY','DZ']],
    ['Taiwan',24,44,'AS',23.6,120.9,-8,'BV',['BM','BN','BO','BP','BQ','BR','BS','BT','BU','BV','BW','BX']],
    ['China',24,33,'AS',36.0,104.0,-8,'BY',['B','BA','BD','BG','BH','BI','BJ','BL','BY']],
    ['Hong Kong',24,44,'AS',22.3,114.2,-8,'VR',['VR']],
    ['Macao',24,44,'AS',22.2,113.5,-8,'XX9',['XX9']],
    ['Korea (South)',25,44,'AS',36.6,128.0,-9,'HL',['DS','DT','HL']],
    ['Korea (North)',25,44,'AS',40.0,127.0,-9,'P5',['HM','P5']],
    ['Japan',25,45,'AS',36.0,138.0,-9,'JA',['7J','7K','7L','7M','7N','7O','8J','8K','8L','8M','8N','JA','JD1','JE','JF','JG','JH','JI','JJ','JK','JL','JM','JN','JO','JP','JQ','JR','JS']],
    ['Ogasawara',25,45,'AS',27.1,142.2,-9,'JD1',['JD1']],
    ['Mongolia',23,32,'AS',47.0,106.0,-8,'JT',['JT','JU','JV']],
    ['Kazakhstan',17,30,'AS',48.0,67.0,-5,'UN',['UN','UO']],
    ['Kyrgyzstan',17,30,'AS',42.0,74.0,-6,'EX',['EX']],
    ['Tajikistan',17,30,'AS',39.0,71.0,-5,'EY',['EY']],
    ['Turkmenistan',21,30,'AS',39.0,59.0,-5,'EZ',['EZ']],
    ['Uzbekistan',17,30,'AS',41.0,64.0,-5,'UK',['UH','UK']],
    ['Azerbaijan',21,29,'AS',40.4,47.9,-4,'4J',['4J','4K']],
    ['Georgia',21,29,'AS',42.2,43.5,-4,'4L',['4L']],
    ['Armenia',21,29,'AS',40.5,44.6,-4,'EK',['EK']],
    ['Spratly Is',26,50,'AS',9.9,114.1,-8,'1S',['1S','9M0','BV9S']],
    ['Paracel Is',24,49,'AS',16.5,112.0,-8,'XS',['XS']],
    // ── Oceania ───────────────────────────────────────────────────────────────
    ['Australia',29,55,'OC',-27.0,133.0,-10,'VK',['AX','VK']],
    ['Lord Howe I',30,60,'OC',-31.5,159.1,-10.5,'VK9L',['VK9L']],
    ['Norfolk I',32,60,'OC',-29.0,167.9,-11.5,'VK9N',['VK9N']],
    ['Macquarie I',30,60,'OC',-54.5,158.9,-11,'VK0M',['VK0M']],
    ['Heard I',29,68,'OC',-53.1,73.5,-5,'VK0H',['VK0H']],
    ['New Zealand',32,60,'OC',-41.3,172.7,-12,'ZL',['ZL','ZM']],
    ['Kermadec Is',32,60,'OC',-29.2,-177.9,12,'ZL8',['ZL8']],
    ['Chatham Is',32,60,'OC',-43.9,-176.5,12.75,'ZL7',['ZL7']],
    ['Antarctica',38,67,'AN',-90.0,0.0,0,'KC4',['3Y','DP0','RI1AN','VK0','ZL5','=4K1A']],
    ['Fiji',28,56,'OC',-17.8,178.0,-12,'3D2',['3D2']],
    ['Rotuma I',28,56,'OC',-12.5,177.1,-12,'3D2/R',['=3D2/R']],
    ['Conway Reef',28,56,'OC',-21.7,174.1,-12,'3D2/C',['=3D2/C']],
    ['Tuvalu',28,65,'OC',-8.5,179.2,-12,'T2',['T2']],
    ['Tokelau Is',31,62,'OC',-9.2,-171.8,11,'ZK3',['ZK3']],
    ['Samoa',32,62,'OC',-13.9,-171.8,11,'5W',['5W']],
    ['Tonga',32,62,'OC',-21.2,-175.2,13,'A3',['A3']],
    ['Niue',32,62,'OC',-19.1,-169.9,11,'E6',['E6','ZK2']],
    ['Cook Is (N)',31,62,'OC',-10.0,-161.0,10,'E5/N',['=E5/N']],
    ['Cook Is (S)',32,62,'OC',-21.2,-159.8,10,'E5/S',['ZK1','E5/S']],
    ['French Polynesia',31,63,'OC',-17.5,-149.5,10,'FO',['FO']],
    ['Marquesas Is',31,63,'OC',-9.5,-139.5,9.5,'FO/M',['FO/M']],
    ['Clipperton I',7,10,'NA',10.3,-109.2,8,'FO/C',['TX0C']],
    ['Austral Is',32,63,'OC',-23.4,-149.5,10,'FO/A',['FO/A']],
    ['Wallis & Futuna Is',28,62,'OC',-13.3,-176.2,12,'FW',['FW']],
    ['New Caledonia',28,56,'OC',-21.5,165.5,-11,'FK',['FK']],
    ['Chesterfield Is',28,56,'OC',-19.9,158.3,-11,'FK/C',['TX3']],
    ['Vanuatu',28,56,'OC',-16.3,167.7,-11,'YJ',['YJ']],
    ['Solomon Is',28,51,'OC',-9.6,160.2,-11,'H4',['H4','H40']],
    ['Temotu',28,51,'OC',-10.7,166.0,-11,'H40',['H40']],
    ['Papua New Guinea',28,51,'OC',-6.1,143.0,-10,'P2',['P2']],
    ['Nauru',31,65,'OC',-0.5,166.9,-12,'C2',['C2']],
    ['Kiribati (Gilbert Is)',31,65,'OC',1.4,173.0,-12,'T30',['T30','T31','T32','T33']],
    ['Kiribati (Phoenix Is)',31,62,'OC',-4.0,-172.0,12,'T31',['T31']],
    ['Kiribati (Line Is)',31,61,'OC',2.0,-157.4,10,'T32',['T32']],
    ['Banaba I',31,65,'OC',-0.9,169.5,-12,'T33',['T33']],
    ['Micronesia',27,65,'OC',7.0,151.0,-11,'V6',['V6']],
    ['Marshall Is',31,65,'OC',9.1,167.4,-12,'V7',['V7']],
    ['Palau',27,64,'OC',7.3,134.5,-9,'T8',['T8']],
    ['Mariana Is',27,64,'OC',15.2,145.7,-10,'KH0',['AH0','KH0','NH0','WH0']],
    ['Guam',27,64,'OC',13.5,144.8,-10,'KH2',['AH2','KH2','NH2','WH2']],
    ['Pitcairn I',32,63,'OC',-25.1,-130.1,8,'VP6',['VP6']],
    ['Easter I',12,63,'SA',-27.2,-109.4,6,'CE0Y',['CE0Y']],
    ['Desventuradas Is',12,14,'SA',-26.3,-80.1,5,'CE0X',['CE0X']],
    ['Juan Fernandez Is',12,14,'SA',-33.6,-80.0,4,'CE0Z',['CE0Z']],
    ['Galapagos Is',10,12,'SA',-0.6,-90.3,6,'HC8',['HC8','HD8']],
    // ── Atlantic misc ────────────────────────────────────────────────────────
    ['St Helena',36,66,'AF',-15.9,-5.7,0,'ZD7',['ZD7']],
    ['Ascension I',36,66,'AF',-7.9,-14.4,0,'ZD8',['ZD8']],
    ['Tristan da Cunha',38,66,'AF',-37.1,-12.3,0,'ZD9',['ZD9']],
    ['Gough I',38,66,'AF',-40.3,-9.9,0,'ZD9',['=ZD9G']],
  ];

  // ── Build lookup indexes ───────────────────────────────────────────────────

  const PREFIX_MAP = {};   // prefix string → entry index
  const EXACT_MAP  = {};   // callsign string → entry index

  CTY_RAW.forEach((entry, idx) => {
    const aliases = entry[8] || [];
    // main prefix is always in aliases, but also index directly
    const mainPfx = entry[7];
    if (!PREFIX_MAP[mainPfx]) PREFIX_MAP[mainPfx] = idx;

    aliases.forEach(alias => {
      if (alias.startsWith('=')) {
        const call = alias.slice(1).toUpperCase();
        if (!EXACT_MAP[call]) EXACT_MAP[call] = idx;
      } else {
        const pfx = alias.toUpperCase();
        if (!PREFIX_MAP[pfx]) PREFIX_MAP[pfx] = idx;
      }
    });
  });

  // ── Lookup helpers ─────────────────────────────────────────────────────────

  function entryToResult(idx) {
    const e = CTY_RAW[idx];
    return {
      country:    e[0],
      cqZone:     e[1],
      ituZone:    e[2],
      continent:  e[3],
      latitude:   e[4],
      longitude:  e[5],
      utcOffset:  e[6],
      mainPrefix: e[7],
    };
  }

  function lookupByPrefix(pfx) {
    pfx = pfx.toUpperCase();
    for (let len = pfx.length; len >= 1; len--) {
      const candidate = pfx.substring(0, len);
      if (PREFIX_MAP[candidate] !== undefined) {
        return entryToResult(PREFIX_MAP[candidate]);
      }
    }
    return null;
  }

  // ── Public: lookupDxcc ─────────────────────────────────────────────────────

  function lookupDxcc(rawCall) {
    if (!rawCall) return null;
    let call = rawCall.toUpperCase().trim();

    // Maritime/aeronautical mobile — no DXCC
    if (call.endsWith('/MM') || call.endsWith('/AM')) return null;

    // Strip portable/mobile suffixes
    const suffixes = ['/QRP', '/LGT', '/P', '/M', '/A', '/B', '/J', '/E', '/T'];
    for (const suf of suffixes) {
      if (call.endsWith(suf)) { call = call.slice(0, -suf.length); break; }
    }

    // Check exact call match first (handles special expeditions)
    if (EXACT_MAP[call] !== undefined) return entryToResult(EXACT_MAP[call]);

    // Handle CALL/PREFIX or PREFIX/CALL
    const parts = call.split('/');
    if (parts.length >= 2) {
      const p1 = parts[0];
      const p2 = parts[parts.length - 1];
      // Try each part, prefer the one giving a valid hit
      const r1 = lookupByPrefix(p1);
      const r2 = lookupByPrefix(p2);
      if (r1 && r2) {
        // Both match: use the shorter part as the portable prefix indicator
        return p1.length <= p2.length ? r1 : r2;
      }
      return r1 || r2;
    }

    return lookupByPrefix(call);
  }

  // ── Public: locatorToLatLon ────────────────────────────────────────────────

  function locatorToLatLon(loc) {
    if (!loc || loc.length < 4) return null;
    loc = loc.toUpperCase();
    const A = loc.charCodeAt(0) - 65;   // field lon
    const B = loc.charCodeAt(1) - 65;   // field lat
    const C = parseInt(loc[2], 10);      // square lon digit
    const D = parseInt(loc[3], 10);      // square lat digit
    if (isNaN(A) || isNaN(B) || isNaN(C) || isNaN(D)) return null;

    let lon = A * 20 - 180 + C * 2;
    let lat = B * 10 -  90 + D;

    if (loc.length >= 6) {
      const E = loc.charCodeAt(4) - 65;  // subsquare lon
      const F = loc.charCodeAt(5) - 65;  // subsquare lat
      lon += E * (2 / 24) + 1 / 24;
      lat += F * (1 / 24) + 0.5 / 24;
    } else {
      lon += 1.0;
      lat += 0.5;
    }

    return { lat, lon };
  }

  // ── Public: calculateQrbAzimuth ────────────────────────────────────────────

  function calculateQrbAzimuth(myLat, myLon, dxLat, dxLon) {
    const R = 6371;
    const toRad = d => d * Math.PI / 180;
    const toDeg = r => r * 180 / Math.PI;

    const lat1 = toRad(myLat), lat2 = toRad(dxLat);
    const dLon = toRad(dxLon - myLon);
    const dLat = toRad(dxLat - myLat);

    const a = Math.sin(dLat / 2) ** 2 +
              Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const qrbKm = Math.round(R * c);

    const y = Math.sin(dLon) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) -
              Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
    let az = toDeg(Math.atan2(y, x));
    az = ((az % 360) + 360) % 360;

    return { qrbKm, azimuthDeg: Math.round(az) };
  }

  // ── Export ─────────────────────────────────────────────────────────────────

  global.DXCC = { lookupDxcc, locatorToLatLon, calculateQrbAzimuth };

}(window));
