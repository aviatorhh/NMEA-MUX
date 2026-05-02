#ifndef DGPS_H_
#define DGPS_H_
#include <avr/pgmspace.h>
#include <stdint.h>

typedef struct {
    uint16_t id;
    const char* name;
} dgps_station_t;

static const char s_zeven[] PROGMEM = "Zeven";
static const char s_helgoland[] PROGMEM = "Helgoland";
static const char s_koblenz[] PROGMEM = "Koblenz";
static const char s_iff_ezheim[] PROGMEM = "Iffezheim";
static const char s_bad_abbach[] PROGMEM = "Bad Abbach";
static const char s_mauken[] PROGMEM = "Mauken";
static const char s_gross_mohrdorf[] PROGMEM = "Gross Mohrdorf";
static const char s_vlieland[] PROGMEM = "Vlieland Phare";
static const char s_hoek[] PROGMEM = "Hoek van Holland";
static const char s_helgoland_bight[] PROGMEM = "Helgoland Bight";
static const char s_borkum[] PROGMEM = "Borkum";
static const char s_wormleighton[] PROGMEM = "Wormleighton";
static const char s_north_foreland[] PROGMEM = "North Foreland";
static const char s_flamborough[] PROGMEM = "Flamborough";
static const char s_lizard[] PROGMEM = "Lizard";
static const char s_nash_point[] PROGMEM = "Nash Point";
static const char s_st_catherine[] PROGMEM = "St. Catherine's";
static const char s_point_lynas[] PROGMEM = "Point Lynas";
static const char s_girdle_ness[] PROGMEM = "Girdle Ness";
static const char s_sumburgh[] PROGMEM = "Sumburgh Head";
static const char s_stirling[] PROGMEM = "Stirling";
static const char s_tory_island[] PROGMEM = "Tory Island";
static const char s_loop_head[] PROGMEM = "Loop Head";
static const char s_mizen_head[] PROGMEM = "Mizen Head";
static const char s_utvaer[] PROGMEM = "Utvær";
static const char s_utsira[] PROGMEM = "Utsira";
static const char s_svinoy[] PROGMEM = "Svinøy";
static const char s_faerder[] PROGMEM = "Færder";
static const char s_lista[] PROGMEM = "Lista";
static const char s_andenes[] PROGMEM = "Andenes";
static const char s_torsvag[] PROGMEM = "Torsvåg";
static const char s_skomvaer[] PROGMEM = "Skomvær";
static const char s_sklinna[] PROGMEM = "Sklinna";
static const char s_halten[] PROGMEM = "Halten";
static const char s_trondheim[] PROGMEM = "Trondheim";
static const char s_vardo[] PROGMEM = "Vardø";
static const char s_fruholmen[] PROGMEM = "Fruholmen";

static const char s_goteborg[] PROGMEM = "Göteborg";
static const char s_otterbacken[] PROGMEM = "Otterbacken";
static const char s_skutskaer[] PROGMEM = "Skutskär";
static const char s_kapellskar[] PROGMEM = "Kapellskär";
static const char s_nynashamn[] PROGMEM = "Nynäshamn";
static const char s_hoburg[] PROGMEM = "Hoburg";
static const char s_kullen[] PROGMEM = "Kullen";
static const char s_bjuroklubb[] PROGMEM = "Bjuröklubb";
static const char s_jarnas[] PROGMEM = "Järnäs";
static const char s_holmsjo[] PROGMEM = "Holmsjö";

static const char s_turku[] PROGMEM = "Turku";
static const char s_porkkala[] PROGMEM = "Porkkala";
static const char s_mantyluoto[] PROGMEM = "Mäntyluoto";
static const char s_marjaniemi[] PROGMEM = "Marjaniemi";
static const char s_kokkola[] PROGMEM = "Kokkola";
static const char s_outokumpu[] PROGMEM = "Outokumpu";
static const char s_puumala[] PROGMEM = "Puumala";
static const char s_savonlinna[] PROGMEM = "Savonlinna";
static const char s_haarajoki[] PROGMEM = "Haarajoki";

static const char s_reykjanes[] PROGMEM = "Reykjanes";
static const char s_bjargtangar[] PROGMEM = "Bjargtangar";
static const char s_radarhofn[] PROGMEM = "Raufarhöfn";
static const char s_djupivogur[] PROGMEM = "Djúpivogur";

static const char s_tarifa[] PROGMEM = "Tarifa";
static const char s_sagres[] PROGMEM = "Sagres";
static const char s_cabo_finisterre[] PROGMEM = "Cabo Finisterre";
static const char s_cabo_machichaco[] PROGMEM = "Cabo Machichaco";
static const char s_cabo_mayor[] PROGMEM = "Cabo Mayor";
static const char s_cabo_penas[] PROGMEM = "Cabo Penas";
static const char s_estaca_bares[] PROGMEM = "Estaca de Bares";

static const char s_cabo_san_sebastian[] PROGMEM = "Cabo San Sebastian";
static const char s_cabo_salou[] PROGMEM = "Cabo Salou";
static const char s_cabo_llobregat[] PROGMEM = "Punta Llobregat";
static const char s_cabo_nao[] PROGMEM = "Cabo de la Nao";
static const char s_cabo_palos[] PROGMEM = "Cabo de Palos";

static const char s_malaga[] PROGMEM = "Malaga";
static const char s_sabinal[] PROGMEM = "Sabinal";
static const char s_la_entallada[] PROGMEM = "La Entallada";

static const char s_capo_ferret[] PROGMEM = "Cap Ferret";
static const char s_cap_bear[] PROGMEM = "Cap Bear";
static const char s_porquerolles[] PROGMEM = "Porquerolles";

static const char s_olonne[] PROGMEM = "Olonne";
static const char s_pont_buis[] PROGMEM = "Pont de Buis";
static const char s_heauville[] PROGMEM = "Heauville";

static const char s_vieste[] PROGMEM = "Vieste";
static const char s_messina[] PROGMEM = "Messina";
static const char s_sellia_marina[] PROGMEM = "Sellia Marina";

static const char s_venstpils[] PROGMEM = "Ventspils";
static const char s_riga[] PROGMEM = "Riga";
static const char s_narva[] PROGMEM = "Narva";
static const char s_ristna[] PROGMEM = "Ristna";

static const char s_odesa[] PROGMEM = "Odesa";
static const char s_zmiinyi[] PROGMEM = "Zmiinyi Island";

static const char s_istanbul[] PROGMEM = "Rumelifeneri";
static const char s_kumkale[] PROGMEM = "Kumkale";

static const char s_alexandria[] PROGMEM = "Alexandria";
static const char s_port_said[] PROGMEM = "Port Said";
static const char s_mersa_matruh[] PROGMEM = "Mersa Matruh";

static const char s_ras_umm_sid[] PROGMEM = "Ras Umm Sid";
static const char s_ras_gharib[] PROGMEM = "Ras Gharib";
static const char s_quseir[] PROGMEM = "Quseir";

static const char s_ras_al_hadd[] PROGMEM = "Ras Al Hadd";
static const char s_ras_madrakah[] PROGMEM = "Ras Madrakah";
static const char s_marbat[] PROGMEM = "Marbat";

static const char s_kuwait[] PROGMEM = "Kuwait";
static const char s_bahrain[] PROGMEM = "Bahrain";
static const char s_abu_dhabi[] PROGMEM = "Abu Dhabi";
static const char s_ras_al_khaimah[] PROGMEM = "Ras Al Khaymah";

static const char s_abashiri[] PROGMEM = "Abashiri";
static const char s_kushiro[] PROGMEM = "Kushiro Saki";
static const char s_shakotan[] PROGMEM = "Shakotan";
static const char s_soya[] PROGMEM = "Soya Misaki";
static const char s_matsumae[] PROGMEM = "Matsumae";

static const char s_inubo[] PROGMEM = "Inubo Saki";
static const char s_kinkasan[] PROGMEM = "Kinkasan";
static const char s_shiriya[] PROGMEM = "Shiriya Saki";
static const char s_hachijo[] PROGMEM = "Hachijo Shima";
static const char s_tokara[] PROGMEM = "Tokara Nakano Shima";
static const char s_miyako[] PROGMEM = "Miyako Shima";
static const char s_yaese[] PROGMEM = "Yaese";
static const char s_seto[] PROGMEM = "Seto";
static const char s_urayasu[] PROGMEM = "Urayasu";
static const char s_nagoya[] PROGMEM = "Nagoya";
static const char s_daio[] PROGMEM = "Daio Saki";
static const char s_ose[] PROGMEM = "Ose Saki";
static const char s_toi[] PROGMEM = "Toi Misaki";
static const char s_tsurugi[] PROGMEM = "Tsurugi Saki";

static const char s_geomundo[] PROGMEM = "Geomundo";
static const char s_palmido[] PROGMEM = "Palmido";
static const char s_yeongdo[] PROGMEM = "Yeongdo";
static const char s_ulleungdo[] PROGMEM = "Ulleungdo";
static const char s_socheongdo[] PROGMEM = "Socheongdo";
static const char s_muju[] PROGMEM = "Muju";
static const char s_seongju[] PROGMEM = "Seongju";
static const char s_pyeongchang[] PROGMEM = "Pyeongchang";
static const char s_chuncheon[] PROGMEM = "Chuncheon";
static const char s_chungju[] PROGMEM = "Chungju";

static const char s_qinhuangdao[] PROGMEM = "Qinhuangdao";
static const char s_laotieshan[] PROGMEM = "Laotieshan";
static const char s_yingkou[] PROGMEM = "Yingkou";
static const char s_beitang[] PROGMEM = "Beitang";
static const char s_dinghai[] PROGMEM = "Dinghai";
static const char s_zhenhaijiao[] PROGMEM = "Zhenhaijiao";
static const char s_fangchenggang[] PROGMEM = "Fangchenggang";
static const char s_sanya[] PROGMEM = "Sanya";
static const char s_naozhoudao[] PROGMEM = "Naozhoudao";
static const char s_shitang[] PROGMEM = "Shitang";
static const char s_lingkun[] PROGMEM = "Lingkun";
static const char s_hao_zhigang[] PROGMEM = "Haozhigang";
static const char s_yanweigan[] PROGMEM = "Yanweigan";
static const char s_sanzaodao[] PROGMEM = "Sanzaodao";

static const char s_houlong[] PROGMEM = "HouLong";

static const char s_cam_ranh[] PROGMEM = "Cam Ranh";
static const char s_mong_cai[] PROGMEM = "Mong Cai";
static const char s_da_nang[] PROGMEM = "Da Nang";
static const char s_nghe_an[] PROGMEM = "Nghe An";

static const char s_singapore[] PROGMEM = "Pulau Satumu";
static const char s_kuching[] PROGMEM = "Kuching";
static const char s_bintulu[] PROGMEM = "Bintulu";
static const char s_bandar_melaka[] PROGMEM = "Bandar Melaka";
static const char s_bagan_datu[] PROGMEM = "Bagan Datuk";
static const char s_kuantan[] PROGMEM = "Kuantan";
static const char s_kuala_besar[] PROGMEM = "Kuala Besar";

static const char s_acushnet[] PROGMEM = "Acushnet";
static const char s_hudson_falls[] PROGMEM = "Hudson Falls";
static const char s_sandy_hook[] PROGMEM = "Sandy Hook";
static const char s_moriches[] PROGMEM = "Moriches";
static const char s_driver[] PROGMEM = "Driver";
static const char s_card_sound[] PROGMEM = "Card Sound";
static const char s_angleton[] PROGMEM = "Angleton";
static const char s_tampa[] PROGMEM = "Tampa";
static const char s_english_turn[] PROGMEM = "English Turn";
static const char s_point_loma[] PROGMEM = "Point Loma";
static const char s_fort_stevens[] PROGMEM = "Fort Stevens";
static const char s_whidbey[] PROGMEM = "Whidbey Island";
static const char s_appleton[] PROGMEM = "Appleton";
static const char s_biorka[] PROGMEM = "Biorka";
static const char s_annette[] PROGMEM = "Annette Island";
static const char s_kodiak[] PROGMEM = "Kodiak";
static const char s_kenai[] PROGMEM = "Kenai";
static const char s_level_island[] PROGMEM = "Level Island";
static const char s_gustavus[] PROGMEM = "Gustavus";
static const char s_potato_point[] PROGMEM = "Potato Point";

static const char s_cardinal[] PROGMEM = "Cardinal";
static const char s_riviere_du_loup[] PROGMEM = "Riviere du Loup";
static const char s_moisie[] PROGMEM = "Moisie";
static const char s_lauzon[] PROGMEM = "Lauzon";
static const char s_st_jean[] PROGMEM = "St. Jean-sur-Richelieu";
static const char s_pt_escuminiac[] PROGMEM = "Pt. Escuminiac";
static const char s_partridge[] PROGMEM = "Partridge Island";
static const char s_fox_island[] PROGMEM = "Fox Island";
static const char s_western_head[] PROGMEM = "Western Head";
static const char s_hartlen_point[] PROGMEM = "Hartlen Point";

static const char s_rio_grande[] PROGMEM = "Rio Grande";
static const char s_paranagua[] PROGMEM = "Paranagua";
static const char s_moela[] PROGMEM = "Moela";
static const char s_canivete[] PROGMEM = "Canivete";
static const char s_calcanhar[] PROGMEM = "Calcanhar";
static const char s_sao_tome[] PROGMEM = "Sao Tome";
static const char s_sao_marcos[] PROGMEM = "Sao Marcos";
static const char s_abrolhos[] PROGMEM = "Abrolhos";
static const char s_sergipe[] PROGMEM = "Sergipe";

static const char s_cayenne[] PROGMEM = "Cayenne";
static const char s_gatun[] PROGMEM = "Gatun";
static const char s_miraflores[] PROGMEM = "Miraflores";
static const char s_unknown[] PROGMEM = "UNKNOWN";

const dgps_station_t dgps_table[] PROGMEM = {

    {6, s_moriches},
    {7, s_moriches},
    {8, s_sandy_hook},
    {9, s_sandy_hook},
    {12, s_driver},
    {13, s_driver},
    {16, s_card_sound},
    {17, s_card_sound},
    {18, s_tampa},
    {19, s_tampa},
    {28, s_english_turn},
    {29, s_english_turn},
    {44, s_acushnet},
    {45, s_acushnet},
    {86, s_odesa},
    {94, s_hudson_falls},
    {95, s_zmiinyi},

    {120, s_bagan_datu},
    {121, s_bagan_datu},
    {122, s_kuantan},
    {123, s_kuantan},
    {124, s_bandar_melaka},
    {125, s_bandar_melaka},
    {126, s_kuala_besar},
    {127, s_kuala_besar},
    {132, s_singapore},
    {133, s_singapore},
    {172, s_appleton},
    {173, s_cam_ranh},
    {175, s_nghe_an},
    {186, s_mong_cai},
    {187, s_mong_cai},
    {199, s_da_nang},
    {200, s_da_nang},

    {262, s_point_loma},
    {263, s_point_loma},
    {272, s_fort_stevens},
    {273, s_fort_stevens},
    {276, s_whidbey},
    {277, s_whidbey},
    {278, s_annette},
    {279, s_annette},
    {280, s_biorka},
    {281, s_biorka},
    {282, s_level_island},
    {283, s_level_island},
    {284, s_gustavus},
    {285, s_gustavus},
    {290, s_potato_point},
    {291, s_potato_point},
    {292, s_kenai},
    {293, s_kenai},
    {294, s_kodiak},
    {295, s_kodiak},

    {308, s_cardinal},
    {309, s_cardinal},
    {316, s_lauzon},
    {317, s_lauzon},
    {318, s_riviere_du_loup},
    {319, s_riviere_du_loup},
    {320, s_moisie},
    {321, s_moisie},
    {326, s_partridge},
    {327, s_partridge},
    {330, s_heauville},
    {331, s_hartlen_point},
    {332, s_pont_buis},
    {333, s_pt_escuminiac},
    {334, s_olonne},
    {335, s_western_head},
    {336, s_capo_ferret},
    {337, s_fox_island},
    {338, s_cap_bear},
    {339, s_porquerolles},

    {350, s_cabo_machichaco},
    {351, s_cabo_mayor},
    {352, s_cabo_penas},
    {353, s_estaca_bares},
    {354, s_cabo_finisterre},
    {356, s_tarifa},
    {357, s_malaga},
    {358, s_sabinal},
    {360, s_cabo_nao},
    {363, s_cabo_san_sebastian},
    {364, s_cabo_salou},
    {365, s_cabo_llobregat},
    {367, s_la_entallada},

    {440, s_alexandria},
    {441, s_alexandria},
    {442, s_port_said},
    {443, s_port_said},
    {444, s_ras_umm_sid},
    {445, s_ras_umm_sid},
    {446, s_ras_gharib},
    {447, s_ras_gharib},
    {448, s_mersa_matruh},
    {449, s_mersa_matruh},
    {450, s_quseir},
    {451, s_quseir},

    {460, s_sao_marcos},
    {461, s_abrolhos},
    {462, s_moela},
    {463, s_canivete},
    {464, s_rio_grande},
    {465, s_sao_tome},
    {467, s_calcanhar},
    {468, s_sergipe},
    {470, s_paranagua},

    {480, s_bahrain},
    {481, s_bahrain},
    {482, s_sagres},   // note: Kuwait conflict resolved → first kept
    {483, s_sagres},
    {484, s_ras_al_khaimah},
    {485, s_ras_al_khaimah},
    {486, s_abu_dhabi},
    {487, s_abu_dhabi},

    {490, s_ras_al_hadd},
    {491, s_ras_al_hadd},
    {492, s_ras_madrakah},
    {493, s_ras_madrakah},
    {494, s_marbat},
    {495, s_marbat},

    {510, s_vieste},
    {511, s_vieste},
    {520, s_messina},
    {521, s_messina},
    {530, s_sellia_marina},

    {545, s_istanbul},
    {546, s_istanbul},
    {547, s_kumkale},
    {548, s_kumkale},

    {600, s_porkkala},
    {601, s_mantyluoto},
    {602, s_puumala},
    {603, s_outokumpu},
    {604, s_turku},
    {605, s_marjaniemi},
    {606, s_kokkola},
    {607, s_haarajoki},
    {609, s_savonlinna},

    {622, s_reykjanes},
    {623, s_reykjanes},
    {624, s_bjargtangar},
    {625, s_bjargtangar},
    {628, s_radarhofn},
    {629, s_radarhofn},
    {630, s_djupivogur},
    {631, s_djupivogur},

    {650, s_hoek},
    {651, s_hoek},
    {655, s_vlieland},
    {656, s_vlieland},

    {660, s_mizen_head},
    {665, s_loop_head},
    {670, s_tory_island},
    {680, s_st_catherine},
    {681, s_lizard},
    {682, s_point_lynas},
    {685, s_sumburgh},
    {686, s_girdle_ness},
    {687, s_flamborough},
    {688, s_north_foreland},
    {689, s_nash_point},
    {691, s_wormleighton},
    {693, s_stirling},

    {720, s_holmsjo},
    {722, s_bjuroklubb},
    {724, s_jarnas},
    {726, s_skutskaer},
    {728, s_kapellskar},
    {730, s_hoburg},
    {732, s_kullen},
    {734, s_nynashamn},
    {736, s_goteborg},
    {738, s_otterbacken},

    {760, s_koblenz},
    {761, s_gross_mohrdorf},
    {762, s_helgoland},
    {763, s_zeven},
    {764, s_iff_ezheim},
    {765, s_bad_abbach},
    {766, s_mauken},

    {780, s_faerder},
    {783, s_lista},
    {785, s_utsira},
    {787, s_utvaer},
    {788, s_svinoy},
    {790, s_halten},
    {791, s_sklinna},
    {793, s_skomvaer},
    {794, s_andenes},
    {796, s_torsvag},
    {797, s_fruholmen},
    {800, s_vardo},
    {801, s_trondheim},
    {802, s_trondheim},

    {821, s_bintulu},
    {822, s_kuching},

    {840, s_ristna},
    {841, s_narva},

    {901, s_miraflores},
    {902, s_miraflores},
    {903, s_gatun},
    {904, s_gatun},

    {986, s_cayenne},
};
const char* dgps_lookup(uint16_t id);
#endif