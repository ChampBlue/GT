import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

def point_in_poly(x: float, y: float, poly_xy) -> bool:
    inside = False
    n = len(poly_xy)
    j = n - 1
    for i in range(n):
        xi, yi = poly_xy[i]
        xj, yj = poly_xy[j]

        if ((yi > y) != (yj > y)):
            x_inter = (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            if x < x_inter:
                inside = not inside
        j = i
    
    return inside

class ZoneMaskServer(Node):
    def __init__(self):
        super().__init__('zone_mask_server')

        self.declare_parameter('resolution', 0.5)       #meter/cell
        self.declare_parameter('width', 400)            #cells
        self.declare_parameter('height', 400)           #cells
        self.declare_parameter('origin_x', -100.0)      #meters
        self.declare_parameter('origin_y', -100.0)      #meters

        self.declare_parameter('lethal_value', 100)     #no go 강도
        self.declare_parameter('publish_rate_hz', 1.0)  #latched라 0도 가능하지만, 디버깅용

        self.res = float(self.get_parameter('resolution').value)
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.ox = float(self.get_parameter('origin_x').value)
        self.oy = float(self.get_parameter('origin_y').value)

        self.lethal = int(self.get_parameter('lethal_value').value)
        self.pub_rate = float(self.get_parameter('publish_rate_hz').value)

        qos_mask = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mask_pub = self.create_publisher(OccupancyGrid, 'keepout_filter_mask', qos_mask)
        self.create_subscription(MarkerArray, '/no_go_zones', self._zones_cb, 10)

        self.polygons = []
        self._dirty = False

        period = 1.0 / max(self.pub_rate, 0.1)
        self.timer = self.create_timer(period, self._publish_if_dirty)

        # 초기 시작 시 빈 마스크 발행 이유 : 구독자가(costmap / keepout_filter)가 언제 붙어도 초기 빈 생태를 받게 하려고.
        # 1) 초기상태를 명시적으로 pub해야함.
        # - 폴리곤이 들어오기 전에도 다음이 필요함
        # keepout_filter는 마스크를 구독하는데, 마스크 토픽이 아무것도 암보내면 경고뜨거나 타이밍에 따라 폴리곤 들어왔을 떄 놓칠 수 있음.
        # 2) TRANSIENT_LOCAK(라치)효과를 초기부터 가지려면 1회 발행이 필요함.
        # - 마지막 메시지를 저장헤 늦게 구독하는 애한테도 주는 개념인데 저장할 마지막 메시지가 있으려면 적어도 한 번은 pub해야함.

        self._publish_mask([])  

    def _zones_cb(self, msg: MarkerArray):
        '''
        visualization_msgs/Marker : rviz에 뭘 그려라라고 보내는 메시지타입.
        점, 선, 화살표, 큐브, 텍스트 증 다양한 형태를 한 메시지 타입으로 표현할 수 있게 만든 "그리기 명령"
        Marker = " 도형 하나 그려라(또는 지워라) " 명령
        MarkerArray = Marker 여러개를 묶어서 한 번에 보내는 컨태이너
        여기선 폴리곤 데이터를 운반하는 포멧으로 재활용
        '''
        polys = []
        for m in msg.markers:
            #Marker type
            # SPHERE, CUBE : 점, 박스
            # LINE_STRIP : 점들을 순서대로 이어서 하나의 폴리라인(연속 선)을 그림
            # LINE_LIST: 점 2개씩 짝지어서 여러 선분 그림
            if m.type != Marker.LINE_STRIP: # pc가 폴리곤을 LINE_STRIP으로 보냈다는 규약을 정했으니, 다른 타입은 무시
                continue
            
            #Marker에는 ADD, DELETE가 있음
            # action = ADD          : 이 마커(id/ns)를 그려라/갱신해라
            # action = DELETE       : 이 마커(id/ns)를 지워라
            # action = DELETEALL    : 해당 ns 또는 전체 다 지워라
            if m.action in (Marker.DELETE, Marker.DELETEALL):   # pc에서 폴리곤 지우면 로봇도 지워야함 그래서 deleteall넣어 보내면 polys=[] 비우고 마스크도 빈 걸로 갱신
                polys = []
                continue

            pts = m.points
            if len(pts) < 3:
                continue
            
            # 닫힘(마지막 == 첫번째)면 마지막 제거
            # pc에서 Marker로 폴리곤을 그릴 떄 시각적으로 "닫힌 도형"처럼 보이게 하려고 points[p0, p1, p2, 03, p0] 이렇게함
            # 즉 마지막에 첫 점을 한 번 더 넣어서 선이 처음으로 돌아가게 함. 
            # 로봇에서 폴리곤 꼭지점 리스트를 내부 계산에 쓰는데, p0가 2번 들어가 잇으면 불필요한 중복점이되고, bbox계산이나 엣지 반복에서 쓸데없는 한 변이 생길 수 있어서 제거

            # abs(...) < 1e-6
            # 점이 논리적으로 같아도 실제 값은 아주 미세하게 다를 수 있기떄문에 0이 아니라 허용오차를 둠(아주 작은 값으로)
            if (abs(pts[0].x - pts[-1].x < 1e-6) and (abs(pts[0].y - pts[-1].y) < 1e-6)):
                pts = pts[:-1]

            poly_xy = [(p.x, p.y) for p in pts]
            if len(poly_xy) >= 3:
                polys.append(poly_xy)

        self.polygons = polys
        self._dirty = True
        self.get_logger().info(f"Receicve polygons : {len(self.polygons)}")

    def _build_grid_msg(self) -> OccupancyGrid:
        '''
        OccupancyGrid() : 2D 격자 지도(그리드) 메시지
        nav_msgs/OccupancyGrid 구성
        1. header
        - frame_id : 이 그리드가 어떤 좌표계 기준인지(보통 map)
        - stamp : 어떤 시점인지
        
        2. info(MapMetaData)
        - resolution : 한 칸(셀)이 실제 세계에서 몇 m인지
        - width, height: 셀 개수(가로/세로)
        - origin : 이 그리드의 (0,0) 셀이 실제 세계에서 어디인지

        3. data
        - 길이 = width*height인 1차원 배열
        - 각 값은 통산 0~100(점유도), 혹은 -1(unknown)
        - 여기선 keepout 마스크로 쓰니 안막는 곳 : 0, 막는 곳 : 99~100
        
        즉, 여기선 map 좌표계 위에 해상도 0.5m짜리 바둑판을 깔고 각 칸에 비용(0~100)을 적어 놓은 데이터
        '''
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = self.res
        m.info.width = self.w
        m.info.height = self.h

        '''
        Pose(origin)는 그 바둑판을 map 위 어디에 둘지를 정해주는 앵커
        아래 origin은 OccupancyGrid의 (0,0) 셀(왼쪽 아래 모서리)이 map에서 어디 좌표인가?
        origin.position.x = -100.0
        origin.position.y = -100.0
        resolution = 0.5
        width=400, height=400

        이때 그리드의 세계 좌표 범위는 -100 ~ 100 즉 map기준으로 200m x 200m 영역을 덮는 마스크
        '''
        origin = Pose()
        origin.position.x = self.ox
        origin.position.y = self.oy
        origin.orientation.w = 1.0

        m.info.origin = origin
        
        return m
    
        #요약
        #map좌표계에 큰 격자판을 깐다 -> 폴리곤이 덮는 칸은 100으로 색칠함. -> 그 격자판을 OccupancyGrid로 pub한다.

    def _publish_mask(self, polygons_xy):
        grid = self._build_grid_msg()
        data = [0] * (self.w * self.h)

        #world => cell
        def w2c(x, y):
            ix = int((x-self.ox) / self.res)
            iy = int((y-self.oy) / self.res)
            return ix, iy
        
        for poly in polygons_xy:
            minx = min(x for x, _ in poly)
            maxx = max(x for x, _ in poly)
            miny = min(y for y, _ in poly)
            maxy = max(y for y, _ in poly)

            x0, y0 = w2c(minx, miny)
            x1, y1 = w2c(maxx, maxy)

            x0 = max(0, min(self.w - 1, x0))
            x1 = max(0, min(self.w - 1, x1))
            y0 = max(0, min(self.h - 1, y0))
            y1 = max(0, min(self.h - 1, y1))

            for iy in range(y0, y1 + 1):
                cy = self.oy + (iy + 0.5) * self.res
                row = iy * self.w
                for ix in range(x0, x1 + 1):
                    cx = self.ox + (ix + 0.5) * self.res
                    if point_in_poly(cx, cy, poly):
                        v = self.lethal
                        if v > data[row + ix]:
                            data[row + ix] = v
            
        grid.data = data
        grid.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(grid)

    def _publish_if_dirty(self):
        if not self._dirty:
            return
        self._dirty = False
        self._publish_mask(self.polygons)





